import os
import sys
import optparse
import random
import networkx as nx
import logging
from enum import Enum
import numpy as np
from scipy.special import bdtr

logging.basicConfig(
    format='%(levelname)s: %(message)s',
    level=logging.INFO,
    filename='output/traci.log',
    filemode='w'
)

# we need to import some python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


from sumolib import checkBinary  # Checks for the binary in environ vars
import traci

from network.config import TOTAL_VEHICLES, TOTAL_PARKING_SPOTS
GRAPH_PICKLED_FILE_LOCATION = 'network/graph.gpickle'
PARKING_NEED_PROBABILITY = 0.5
# TOTAL_CARS_IN_THE_NETWORK = TOTAL_VEHICLES
TOTAL_CARS_IN_THE_NETWORK = 2
# TOTAL_CARS_PARKED_TARGET should atmost be TOTAL_PARKING_SPOTS - 1
# TOTAL_CARS_PARKED_TARGET = int(TOTAL_PARKING_SPOTS / 8)
TOTAL_CARS_PARKED_TARGET = 1
parkedVehicles = set()

PHEROMONE_CONTRIBUTION_COEFFICIENT = 10
PHEROMONE_DECAY_COEFFICIENT = 0.9
COOLDOWN_PERIOD_AFTER_RELEASED_FROM_PARKING = 50  # in time steps


def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                          default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


class VehicleState(Enum):
    MOVING_AND_MAY_NEED_TO_PARK = 0
    SCHEDULED_TO_PARK = 1
    PARKED = 2
    MOVING_AND_NOT_LOOKING_TO_PARK = 3


def calc_prob_of_free_spots(edgeID, availableParkingSpaces, movingVehiclesCount):
    if availableParkingSpaces == 0:
        return 0
    if availableParkingSpaces > movingVehiclesCount:
        return 1
    k = availableParkingSpaces - 1
    n = movingVehiclesCount
    p = PARKING_NEED_PROBABILITY
    edgeParkingProbability = bdtr(k, n, p)
    return edgeParkingProbability


def random_routing_from_lane_neighbors(vehID):
    currentLaneID = traci.vehicle.getLaneID(vehID)
    if currentLaneID in G and not G.nodes[currentLaneID]['is_internal']:
        nextLaneID = random.choice(list(G[currentLaneID].keys()))
        traci.vehicle.setRoute(vehID, [G.nodes[currentLaneID]['parent_edge'], G.nodes[nextLaneID]['parent_edge']])


def random_routing_from_edge_neighbors(vehID, edges):
    currentEdgeID = traci.vehicle.getRoadID(vehID)
    if edges.get(currentEdgeID):
        nextEdgeID = random.choice(list(edges[currentEdgeID]['neighbors']))
        traci.vehicle.setRoute(vehID, [currentEdgeID, nextEdgeID])


def get_all_edges(G):
    edgeInfo = {}
    for laneID, datadict in G.nodes.items():
        if not datadict['is_internal']:
            edgeID = datadict['parent_edge']
            if edgeInfo.get(edgeID) is None:
                edgeInfo[edgeID] = {'parking_areas': [],
                                    'lanes': [], 'neighbors': set()}
            edgeInfo[edgeID]['lanes'].append(laneID)

            for neighborLane in list(G[laneID].keys()):
                neighborEdge = G.nodes[neighborLane]['parent_edge']
                edgeInfo[edgeID]['neighbors'].add(neighborEdge)

            for parkingArea in datadict['parking_areas']:
                capacity = int(traci.simulation.getParameter(
                    parkingArea['@id'], "parkingArea.capacity"))
                edgeInfo[edgeID]['parking_areas'].append({'id': parkingArea['@id'],
                                                          'capacity': capacity, 'occupancy': 0})
    return edgeInfo


def calculate_turn_probability_from_edge_neighbors(edgeInfo):
    turnProbability = {}
    for edgeID, info in edgeInfo.items():
        pheromoneSum = 0
        for neighbor in info['neighbors']:
            pheromoneSum += edgeInfo[neighbor]['pheromone']
        turnProbability[edgeID] = {}
        for neighbor in info['neighbors']:
            turnProbability[edgeID][neighbor] = edgeInfo[neighbor]['pheromone'] / pheromoneSum
    return turnProbability


def calculate_turn_probability_from_lane_neighbors(laneID, pheromoneLevels):
    pheromoneSum = 0
    pheromone = {}
    for neighborLane in list(G[laneID].keys()):
        neighborEdge = G.nodes[neighborLane]['parent_edge']
        pheromone[neighborLane] = pheromoneLevels[neighborEdge]
        pheromoneSum += pheromone[neighborLane]

    turnProbability = {}
    for neighborLane in list(G[laneID].keys()):
        turnProbability[neighborLane] = pheromone[neighborLane] / pheromoneSum
    return turnProbability


def pheromone_based_routing_from_lane_neighbors(vehID, pheromoneLevels):
    currentLaneID = traci.vehicle.getLaneID(vehID)
    if G.nodes.get(currentLaneID) and not G.nodes[currentLaneID]['is_internal']:
        turnProbability = calculate_turn_probability_from_lane_neighbors(currentLaneID, pheromoneLevels)
        if currentLaneID in G and not G.nodes[currentLaneID]['is_internal']:
            neighborLanes = list(G[currentLaneID].keys())
            probabilities = [0] * len(neighborLanes)
            for i in range(len(neighborLanes)):
                neighborLane = neighborLanes[i]
                probabilities[i] = turnProbability[neighborLane]
            nextLaneID = np.random.choice(neighborLanes, p=probabilities)
            traci.vehicle.setRoute(
                vehID, [G.nodes[currentLaneID]['parent_edge'], G.nodes[nextLaneID]['parent_edge']])


def get_available_parking_spaces(currentEdgeID, edges):
    availableParkingSpaces = 0
    for parkingArea in edges[currentEdgeID]['parking_areas']:
        availableParkingSpaces += parkingArea['capacity'] - \
            parkingArea['occupancy']
        logging.info("Parking Area: {}, capacity: {}, occupancy: {}".format(parkingArea['id'],
                                                                            parkingArea['capacity'], parkingArea['occupancy']))
    return availableParkingSpaces


def unparkARandomVehicle(travelInfo, vehicleToNotRemove):
    modifiedParkedVehiclesSet = parkedVehicles.copy()
    modifiedParkedVehiclesSet.remove(vehicleToNotRemove)
    vehIDToRemove = random.choice(list(modifiedParkedVehiclesSet))

    # double check that this vehicle is really parked
    if traci.vehicle.isStoppedParking(vehIDToRemove) and travelInfo[vehIDToRemove]['vehicle_state'] == VehicleState.PARKED:
        try:
            traci.vehicle.setParkingAreaStop(vehIDToRemove, travelInfo[vehIDToRemove]['parked_area_id'], duration=0)
        except traci.exceptions.TraCIException as e:
            logging.warning(e)
        except Exception as e:
            logging.error(e)
        else:
            parkedVehicles.remove(vehIDToRemove)
            travelInfo[vehIDToRemove]['vehicle_state'] = VehicleState.MOVING_AND_NOT_LOOKING_TO_PARK
            travelInfo[vehIDToRemove]['parked_area_id'] = None
            logging.info('Vehicle {} has been scheduled to unpark from {}.'.format(vehIDToRemove, travelInfo[vehIDToRemove]['parked_area_id']))


def addPheromonesToTheLastPath(vehID, travelInfo, newPheromones):
    # get the edges travelled by the vehicle since it last got parked
    vehRoute = traci.vehicle.getRoute(vehID)

    # add pheromones on the travelled path
    pathLength = travelInfo[vehID]['finish_distance'] - travelInfo[vehID]['start_distance']
    if pathLength == 0:
        raise Exception("Path length is 0")
    for i in range(travelInfo[vehID]['start_route_id'], travelInfo[vehID]['finish_route_id'] + 1):
        edgeID = vehRoute[i]
        if newPheromones.get(edgeID) is None:
            newPheromones[edgeID] = 0
        newPheromones[edgeID] += 1.0 / pathLength


def markVehicleAsParked(vehID, travelInfo, newPheromones):
    # update the parking details
    travelInfo[vehID]['parked_area_id'] = travelInfo[vehID]['scheduled_parking_area']['id']
    travelInfo[vehID]['scheduled_parking_area']['occupancy'] += 1
    travelInfo[vehID]['scheduled_parking_area'] = None
    parkedVehicles.add(vehID)

    # note down the distance reading
    travelInfo[vehID]['finish_distance'] = travelInfo[vehID]['distance_travelled']

    # mark the current position on the route
    currentIndexOnRoute = traci.vehicle.getRouteIndex(vehID)
    travelInfo[vehID]['finish_route_id'] = currentIndexOnRoute

    addPheromonesToTheLastPath(vehID, travelInfo, newPheromones)
    travelInfo[vehID]['vehicle_state'] = VehicleState.PARKED


def try_to_schedule_a_parking(edges, currentEdgeID, vehID, travelInfo):
    # try to park the car on some parking area on this edge with free areas
    for parkingArea in edges[currentEdgeID]['parking_areas']:
        logging.debug("Parking Area: {}, capacity: {}, "
                      "occupancy: {}".format(parkingArea['id'], parkingArea['capacity'], parkingArea['occupancy']))
        if parkingArea['occupancy'] < parkingArea['capacity']:
            try:
                traci.vehicle.setParkingAreaStop(vehID, parkingArea['id'], duration=100000)
            except traci.exceptions.TraCIException as e:
                logging.warning(e)
            except Exception as e:
                logging.error(e)
            else:
                # this vehcile may get parked in the future iterations
                # change state and break the loop as we don't need to check other parking areas on this edge
                travelInfo[vehID]['vehicle_state'] = VehicleState.SCHEDULED_TO_PARK
                travelInfo[vehID]['scheduled_parking_area'] = parkingArea
                logging.info("Vehicle {} has been scheduled to park at {}.".format(vehID, parkingArea['id']))
                break


def try_to_park_if_needed(vehID, currentEdgeID, edges, travelInfo):
    parkingNeedToss = random.random()
    logging.debug("VehID: {}, EdgeID: {}, PARKING_NEED: {}, "
                  "parkingNeedToss: {}".format(vehID, currentEdgeID, PARKING_NEED_PROBABILITY, parkingNeedToss))
    if parkingNeedToss < PARKING_NEED_PROBABILITY:
        movingVehiclesCount = traci.edge.getLastStepVehicleNumber(currentEdgeID)
        availableParkingSpaces = get_available_parking_spaces(currentEdgeID, edges)
        edgeParkingProbability = calc_prob_of_free_spots(currentEdgeID, availableParkingSpaces, movingVehiclesCount)
        parkingTossValue = random.random()
        logging.debug("VehID: {}, EdgeID: {}, parkingGurantee: {}, parkingTossValue: {}".format(
            vehID, currentEdgeID, edgeParkingProbability, parkingTossValue))
        if parkingTossValue < edgeParkingProbability:
            try_to_schedule_a_parking(edges, currentEdgeID, vehID, travelInfo)


def updatePheromoneLevels(newPheromones, pheromoneLevels):
    for edgeID, pheromoneLevel in newPheromones.items():
        pheromoneLevels[edgeID] = PHEROMONE_DECAY_COEFFICIENT * pheromoneLevels[edgeID]
        pheromoneLevels[edgeID] += PHEROMONE_CONTRIBUTION_COEFFICIENT * pheromoneLevel


# contains TraCI control loop
def run(G, max_steps=500):
    # initialize the simulation wide variables
    edges = get_all_edges(G)
    pheromoneLevels = {}
    for edgeID in edges:
        pheromoneLevels[edgeID] = 1
    travelInfo = {}

    step = 1
    while traci.simulation.getMinExpectedNumber() > 0 and step < max_steps:
        traci.simulationStep()
        logging.info("Simulation Step: {}".format(step))
        newPheromones = {}  # sum of pheromones added by any vehicles in this time step

        for vehID in traci.vehicle.getIDList():
            if travelInfo.get(vehID) is None:
                travelInfo[vehID] = {'vehicle_state': VehicleState.MOVING_AND_MAY_NEED_TO_PARK, 'parked_area_id': None,
                                     'scheduled_parking_area': None, 'start_distance': 0, 'finish_distance': None,
                                     'start_route_id': 0, 'finish_route_id': None, 'cooldown_counter': 0,
                                     'distance_travelled': 0}
                logging.info("Vehicle {} is moving and needs to park.".format(vehID))

            # the value given by traci.vehicle.getDistance() becomes negative when the vehicle is parked
            # so we should keep track of this value ourselves whenever its positive or to be safe more than the last reading
            d = traci.vehicle.getDistance(vehID)
            if d > travelInfo[vehID]['distance_travelled']:
                travelInfo[vehID]['distance_travelled'] = d

            currentEdgeID = traci.vehicle.getRoadID(vehID)
            if edges.get(currentEdgeID) is None:
                continue
            if travelInfo[vehID]['vehicle_state'] == VehicleState.MOVING_AND_MAY_NEED_TO_PARK or travelInfo[vehID]['vehicle_state'] == VehicleState.SCHEDULED_TO_PARK:
                pheromone_based_routing_from_lane_neighbors(vehID, pheromoneLevels)
            else:
                random_routing_from_lane_neighbors(vehID)

            logging.info('Vehicle {}: {}, distance={}'.format(vehID, travelInfo[vehID]['vehicle_state'].name, travelInfo[vehID]['distance_travelled']))

            if travelInfo[vehID]['vehicle_state'] == VehicleState.SCHEDULED_TO_PARK:
                if traci.vehicle.isStoppedParking(vehID):
                    logging.info("Vehicle {} which was scheduled to park has stopped at {}.".format(vehID, travelInfo[vehID]['scheduled_parking_area']['id']))
                    markVehicleAsParked(vehID, travelInfo, newPheromones)
                    logging.info("Vehicle {} has been marked as parked".format(vehID))
                    if len(parkedVehicles) > TOTAL_CARS_PARKED_TARGET:
                        logging.info('Parked limit exceeded, unparking a vehicle other than {}.'.format(vehID))
                        unparkARandomVehicle(travelInfo, vehID)

            if travelInfo[vehID]['vehicle_state'] == VehicleState.PARKED:
                continue

            if travelInfo[vehID]['vehicle_state'] == VehicleState.MOVING_AND_NOT_LOOKING_TO_PARK:
                travelInfo[vehID]['cooldown_counter'] += 1
                if travelInfo[vehID]['cooldown_counter'] == COOLDOWN_PERIOD_AFTER_RELEASED_FROM_PARKING:
                    logging.info('Cooldown period of vehicle {} has ended.'.format(vehID))
                    logging.info("Vehicle {} is moving and needs to park.".format(vehID))
                    travelInfo[vehID]['cooldown_counter'] = 0
                    travelInfo[vehID]['vehicle_state'] = VehicleState.MOVING_AND_MAY_NEED_TO_PARK
                    travelInfo[vehID]['start_distance'] = travelInfo[vehID]['distance_travelled']
                    travelInfo[vehID]['finish_distance'] = None
                    travelInfo[vehID]['start_route_id'] = traci.vehicle.getRouteIndex(vehID)
                    travelInfo[vehID]['finish_route_id'] = None

            if travelInfo[vehID]['vehicle_state'] == VehicleState.MOVING_AND_MAY_NEED_TO_PARK:
                try_to_park_if_needed(vehID, currentEdgeID, edges, travelInfo)

        updatePheromoneLevels(newPheromones, pheromoneLevels)
        step += 1

    traci.close()
    sys.stdout.flush()


if __name__ == "__main__":
    options = get_options()

    # check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    G = nx.read_gpickle(GRAPH_PICKLED_FILE_LOCATION)

    # traci starts sumo as a subprocess and then this script connects and runs
    traci.start([sumoBinary, "-c", "network/aco.sumocfg",
                             "--tripinfo-output", "output/tripinfo.xml"])
    run(G, max_steps=10000)

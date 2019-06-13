import os
import sys
import optparse
import random
import networkx as nx
import logging
import numpy as np
from scipy.special import bdtr

logging.basicConfig(
    format='%(levelname)s: %(message)s',
    level=logging.DEBUG,
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


# def getAvailableParkingSpots(edge):
#     return G.edges[edge]['parking_capacity'] - G.edges[edge]['parked_vehicles_count']


# def getParkedVehcilesCount(edge):
#     return G.edges[edge]['parked_vehicles_count']


# def incrementParkedCount(edge):
#     G.edges[edge]['parked_vehicles_count'] += 1


# def decrementParkedCount(edge):
#     G.edges[edge]['parked_vehicles_count'] -= 1


vehicleTravelInfo = {}


def calculate_probability_to_get_free_parking(edgeID, availableParkingSpaces, movingVehiclesCount):
    if availableParkingSpaces == 0:
        return 0
    if availableParkingSpaces > movingVehiclesCount:
        return 1
    k = availableParkingSpaces - 1
    n = movingVehiclesCount
    p = PARKING_NEED_PROBABILITY
    edgeParkingProbability = bdtr(k, n, p)
    return edgeParkingProbability


def random_routing_from_lane_neighbors(vehID, step):
    currentLaneID = traci.vehicle.getLaneID(vehID)
    if currentLaneID in G and not G.nodes[currentLaneID]['is_internal']:
        nextLaneID = random.choice(list(G[currentLaneID].keys()))
        logging.debug("{} {} {} {}".format(
            step, vehID, currentLaneID, nextLaneID))
        traci.vehicle.setRoute(
            vehID, [G.nodes[currentLaneID]['parent_edge'], G.nodes[nextLaneID]['parent_edge']])


def random_routing_from_edge_neighbors(vehID, edges, step):
    currentEdgeID = traci.vehicle.getRoadID(vehID)
    if edges.get(currentEdgeID):
        nextEdgeID = random.choice(list(edges[currentEdgeID]['neighbors']))
        logging.debug("{} {} {} {}".format(
            step, vehID, currentEdgeID, nextEdgeID))
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


def pheromone_based_routing(vehID, step, pheromoneLevels):
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
            logging.debug("{} {} {} {}".format(
                step, vehID, currentLaneID, nextLaneID))
            traci.vehicle.setRoute(
                vehID, [G.nodes[currentLaneID]['parent_edge'], G.nodes[nextLaneID]['parent_edge']])
    else:
        logging.warning('Vehicle {} is on lane {}.'.format(vehID, currentLaneID))


def get_available_parking_spaces(currentEdgeID, edges):
    availableParkingSpaces = 0
    for parkingArea in edges[currentEdgeID]['parking_areas']:
        availableParkingSpaces += parkingArea['capacity'] - \
            parkingArea['occupancy']
        logging.info("Parking Area: {}, capacity: {}, occupancy: {}".format(parkingArea['id'],
                                                                            parkingArea['capacity'], parkingArea['occupancy']))
    return availableParkingSpaces


# contains TraCI control loop
def run(G, max_steps=500):
    step = 1
    edges = get_all_edges(G)
    pheromoneLevels = {}
    for edgeID in edges:
        pheromoneLevels[edgeID] = 1

    travelInfo = {}

    while traci.simulation.getMinExpectedNumber() > 0 and step < max_steps:
        traci.simulationStep()
        newPheromones = {}
        print("Simulation Step: {}".format(step))
        for vehID in traci.vehicle.getIDList():
            if travelInfo.get(vehID) is None:
                travelInfo[vehID] = {'is_parked': False, 'parking_area_id': None, 'start_distance': 0,
                                     'finish_distance': None, 'overall_route': traci.vehicle.getRoute(vehID),
                                     'start_route_id': 0, 'finish_route_id': None}
            if travelInfo[vehID]['is_parked'] is True:
                continue
            currentEdgeID = traci.vehicle.getRoadID(vehID)
            if edges.get(currentEdgeID):
                # random_routing_from_lane_neighbors(vehID, step)
                pheromone_based_routing(vehID, step, pheromoneLevels)
                parkingNeedToss = random.random()
                logging.info("VehID: {}, EdgeID: {}, PARKING_NEED: {}, parkingNeedToss: {}".format(vehID, edgeID,
                                                                                                   PARKING_NEED_PROBABILITY,
                                                                                                   parkingNeedToss))
                if parkingNeedToss < PARKING_NEED_PROBABILITY:
                    movingVehiclesCount = traci.edge.getLastStepVehicleNumber(
                        currentEdgeID)
                    availableParkingSpaces = get_available_parking_spaces(
                        currentEdgeID, edges)
                    edgeParkingProbability = calculate_probability_to_get_free_parking(currentEdgeID,
                                                                                       availableParkingSpaces,
                                                                                       movingVehiclesCount)
                    parkingTossValue = random.random()
                    logging.info("VehID: {}, EdgeID: {}, parkingGurantee: {}, parkingTossValue: {}".format(
                        vehID, edgeID, edgeParkingProbability, parkingTossValue))
                    if parkingTossValue < edgeParkingProbability:
                        # try to park the car on some parking area on this edge
                        for parkingArea in edges[currentEdgeID]['parking_areas']:
                            # logging.info("Parking Area: {}, capacity: {}, occupancy: {}".format(parkingArea['id'], parkingArea['capacity'], occupancy))
                            if parkingArea['occupancy'] < parkingArea['capacity']:
                                try:
                                    traci.vehicle.setParkingAreaStop(vehID, parkingArea['id'], duration=100000)
                                except traci.exceptions.TraCIException as e:
                                    logging.warning(e)
                                except Exception as e:
                                    logging.error(e)
                                else:
                                    # Problem: We should only execute the below steps if we are sure that the car has parked.
                                    # If some error occurs and the car doesn't get parked it will stay still as we don't search
                                    # for parking areas nor route the vehicles which are parked
                                    # maybe routing parked vehicles also could solve that problem but marking start and finish
                                    # values for a car which isn't actually parked, and the parkedVehicles set is used to remove
                                    # a car. So, check for isStoppedParking() value before executing these steps. And execute this 
                                    # code every iteration if the vehicle is scheduled to park. And once executed remove the vehicle
                                    # from the scheduled vehicles to park.
                                    travelInfo[vehID]['is_parked'] = True
                                    travelInfo[vehID]['parking_area_id'] = parkingArea['id']
                                    parkingArea['occupancy'] += 1
                                    parkedVehicles.add(vehID)
                                    # note down the distance reading
                                    travelInfo[vehID]['finish_distance'] = traci.vehicle.getDistance(vehID)
                                    pathLength = travelInfo[vehID]['finish_distance'] - travelInfo[vehID]['start_distance']

                                    # get the edges travelled by the vehicle since it last got parked
                                    vehRoute = traci.vehicle.getRoute(vehID)
                                    currentIndexOnRoute = traci.vehicle.getRouteIndex(vehID)
                                    travelInfo[vehID]['finish_route_id'] = currentIndexOnRoute
                                    # add pheromones on the travelled path
                                    logging.info("VehID: {} startDistance: {} finishDistance: {}".format(vehID, travelInfo[vehID]['start_distance'], travelInfo[vehID]['finish_distance']))
                                    if pathLength == 0:
                                        # TODO if a vehicle is released from parking, 
                                        # make it wait for sometime, before it starts looking for parking
                                        # otherwise it would choose the same parking lot, resulting in 0 or very small 
                                        # path lengths which would give weird pheromone values
                                        pathLength = 1
                                    for i in range(travelInfo[vehID]['start_route_id'], travelInfo[vehID]['finish_route_id'] + 1):
                                        edgeID = vehRoute[i]
                                        if newPheromones.get(edgeID) is None:
                                            newPheromones[edgeID] = 1.0 / pathLength
                                        else:
                                            newPheromones[edgeID] += 1.0 / pathLength

                                    if len(parkedVehicles) == TOTAL_CARS_PARKED_TARGET:
                                        # remove a parked car before adding one
                                        vehIDToRemove = random.choice(list(parkedVehicles))
                                        try:
                                            traci.vehicle.setParkingAreaStop(vehIDToRemove, travelInfo[vehIDToRemove]['parking_area_id'], duration=0)
                                        except traci.exceptions.TraCIException as e:
                                            logging.warning(e)
                                        except Exception as e:
                                            logging.error(e)
                                        else:
                                            parkedVehicles.remove(vehIDToRemove)
                                            travelInfo[vehIDToRemove]['is_parked'] = False
                                            travelInfo[vehIDToRemove]['parking_area_id'] = None
                                            travelInfo[vehIDToRemove]['start_distance'] = travelInfo[vehIDToRemove]['finish_distance']
                                            travelInfo[vehIDToRemove]['finish_distance'] = None
                                            travelInfo[vehIDToRemove]['start_route_id'] = traci.vehicle.getRouteIndex(vehIDToRemove)
                                            travelInfo[vehIDToRemove]['finish_route_id'] = None
                                    break
        for edgeID, pheromoneLevel in newPheromones.items():
            pheromoneLevels[edgeID] = PHEROMONE_DECAY_COEFFICIENT * pheromoneLevels[edgeID]
            pheromoneLevels[edgeID] += PHEROMONE_CONTRIBUTION_COEFFICIENT * pheromoneLevel
        step += 1

    traci.close()
    sys.stdout.flush()


def getTotalParkingSpots():
    sum = 0
    for laneID, datadict in G.nodes.items():
        if datadict.get('parking_capacity'):
            sum += datadict['parking_capacity']
    return sum


if __name__ == "__main__":
    options = get_options()

    # check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    G = nx.read_gpickle(GRAPH_PICKLED_FILE_LOCATION)
    totalParkingSpots = getTotalParkingSpots()

    # traci starts sumo as a subprocess and then this script connects and runs
    traci.start([sumoBinary, "-c", "network/aco.sumocfg",
                             "--tripinfo-output", "output/tripinfo.xml"])
    run(G, max_steps=10000)

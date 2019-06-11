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

GRAPH_PICKLED_FILE_LOCATION = 'network/graph.gpickle'
PARKING_NEED_PROBABILITY = 0.5
# random.seed(0)


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


def calculate_probability_to_get_free_parking(edgeInfo):
    edgeParkingProbability = {}
    for edgeID, info in edgeInfo.items():
        k = info['available_parking_spaces'] - 1
        n = info['moving_vehicles_count']
        p = PARKING_NEED_PROBABILITY
        edgeParkingProbability[edgeID] = bdtr(k, n, p)
    return edgeParkingProbability


def get_edge_information(G):
    edgeInfo = {}
    for laneID, datadict in G.nodes.items():
        if not datadict['is_internal']:
            edgeID = datadict['parent_edge']
            if edgeInfo.get(edgeID) is None:
                edgeInfo[edgeID] = {'pheromone': 1, 'moving_vehicles_count': 0, 'parked_vehicles_count': 0,
                                    'parking_capacity': 0, 'parking_areas': [], 'lanes': [], 'neighbors': set()}
            edgeInfo[edgeID]['lanes'].append(laneID)

            for neighborLane in list(G[laneID].keys()):
                neighborEdge = G.nodes[neighborLane]['parent_edge']
                edgeInfo[edgeID]['neighbors'].add(neighborEdge)

            edgeInfo[edgeID]['moving_vehicles_count'] += traci.lane.getLastStepVehicleNumber(laneID)
            for parkingArea in datadict['parking_areas']:
                edgeInfo[edgeID]['parking_areas'].append(parkingArea['@id'])
                edgeInfo[edgeID]['parked_vehicles_count'] += int(traci.simulation.getParameter(parkingArea['@id'],
                                                                                               "parkingArea.occupancy"))
                # alternatively use parkingArea.capacity
                edgeInfo[edgeID]['parking_capacity'] += int(parkingArea['@roadsideCapacity'])
            edgeInfo[edgeID]['available_parking_spaces'] = edgeInfo[edgeID]['parking_capacity'] - edgeInfo[edgeID]['parked_vehicles_count']
    return edgeInfo


def calculate_turn_probability_from_lane_neighbors(edgeInfo):
    turnProbability = {}
    for laneID, laneInfo in G.nodes.items():
        pheromone = {}
        pheromoneSum = 0
        for neighborLane in list(G[laneID].keys()):
            neighborEdge = G.nodes[neighborLane]['parent_edge']
            pheromone[neighborLane] = edgeInfo[neighborEdge]['pheromone']
            pheromoneSum += pheromone[neighborLane]

        turnProbability[laneID] = {}
        for neighborLane in list(G[laneID].keys()):
            turnProbability[laneID][neighborLane] = pheromone[neighborLane] / pheromoneSum
    return turnProbability


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


def pheromone_based_routing(vehID, step, turnProbability):
    currentLaneID = traci.vehicle.getLaneID(vehID)
    if currentLaneID in G and not G.nodes[currentLaneID]['is_internal']:
        neighborLanes = list(G[currentLaneID].keys())
        probabilities = [0] * len(neighborLanes)
        for i in range(len(neighborLanes)):
            neighborLane = neighborLanes[i]
            probabilities[i] = turnProbability[currentLaneID][neighborLane]
        nextLaneID = np.random.choice(neighborLanes, p=probabilities)
        logging.debug("{} {} {} {}".format(step, vehID, currentLaneID, nextLaneID))
        traci.vehicle.setRoute(vehID, [G.nodes[currentLaneID]['parent_edge'], G.nodes[nextLaneID]['parent_edge']])


def random_routing_from_lane_neighbors(vehID, step):
    currentLaneID = traci.vehicle.getLaneID(vehID)
    if currentLaneID in G and not G.nodes[currentLaneID]['is_internal']:
        nextLaneID = random.choice(list(G[currentLaneID].keys()))
        logging.debug("{} {} {} {}".format(step, vehID, currentLaneID, nextLaneID))
        traci.vehicle.setRoute(vehID, [G.nodes[currentLaneID]['parent_edge'], G.nodes[nextLaneID]['parent_edge']])


def random_routing_from_edge_neighbors(vehID, edgeInfo, step):
    currentEdgeID = traci.vehicle.getRoadID(vehID)
    if edgeInfo.get(currentEdgeID):
        nextEdgeID = random.choice(list(edgeInfo[currentEdgeID]['neighbors']))
        logging.debug("{} {} {} {}".format(step, vehID, currentEdgeID, nextEdgeID))
        traci.vehicle.setRoute(vehID, [currentEdgeID, nextEdgeID])


# contains TraCI control loop
def run(max_steps=500):
    step = 1
    while traci.simulation.getMinExpectedNumber() > 0 and step < max_steps:
        traci.simulationStep()
        print("Simulation Step: {}".format(step))

        edgeInfo = get_edge_information(G)
        edgeParkingProbability = calculate_probability_to_get_free_parking(edgeInfo)
        turnProbability = calculate_turn_probability_from_lane_neighbors(edgeInfo)

        for vehID in traci.vehicle.getIDList():
            # random_routing_from_lane_neighbors(vehID, step)
            # random_routing_from_edge_neighbors(vehID, edgeInfo, step)
            pheromone_based_routing(vehID, step, turnProbability)

        # if vehicleTravelInfo.get(vehID) is None:
        #     vehicleTravelInfo[vehID] = {'start_distance_reading': 0, 'finish_distance_reading': None, 'overall_route': traci.vehicle.getRoute(vehID), 'start_route_id': 0, 'finish_route_id': None}
        # if random.random() < PARKING_NEED_PROBABILITY:
        #     currentEdgeID = traci.vehicle.getRoadID(vehID)

        #     # the vehicle may be on a junction or an edge
        #     if edgeDict.get(currentEdgeID):
        #         currentEdge = edgeDict[currentEdgeID]
        #         availableParkingSpots = getAvailableParkingSpots(currentEdge)
        #         if availableParkingSpots > 0:
        #             # park the vehicle here
        #             # traci.vehicle.setParkingAreaStop(vehID, 'parkingArea_gneE0_0_1', duration=100000)
        #             vehicleTravelInfo[vehID]['finish_distance_reading'] = traci.vehicle.getDistance(vehID)
        #             print("Vehicle {} covered {} distance before parking.".format(vehID, vehicleTravelInfo[vehID]['finish_distance_reading'] - vehicleTravelInfo[vehID]['start_distance_reading']))
        #             vehRoute = traci.vehicle.getRoute(vehID)
        #             vehRouteIndex = vehRoute[traci.vehicle.getRouteIndex(vehID)]
        #             print("Route: {}".format(vehRoute))
        #             print("Vehicle is at index {}, which is edge {} on this route.".format(vehRouteIndex, vehRoute[vehRouteIndex]))
        #             incrementParkedCount(currentEdge)

        #             # randomly choose an edge which has some vehicle parked in it and remove a vehicle from it
        #             candidateList = []
        #             for edgeID, edge in edgeDict.items():
        #                 if edgeID != currentEdgeID and getParkedVehcilesCount(edge) > 0:
        #                     candidateList.append(edge)
        #             randomEdge = random.choice(candidateList)
        #             decrementParkedCount(randomEdge)

        #             # spawn a new vehicle here, so that moving_vehicles_count doesn't change, update the distance readings
        #             vehicleTravelInfo[vehID]['start_distance_reading'] = vehicleTravelInfo[vehID]['finish_distance_reading']
        #             vehicleTravelInfo[vehID]['finish_distance_reading'] = None
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

    global G
    G = nx.read_gpickle(GRAPH_PICKLED_FILE_LOCATION)
    totalParkingSpots = getTotalParkingSpots()

    # traci starts sumo as a subprocess and then this script connects and runs
    traci.start([sumoBinary, "-c", "network/aco.sumocfg",
                             "--tripinfo-output", "output/tripinfo.xml"])
    run(max_steps=1000)

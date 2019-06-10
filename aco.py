import os
import sys
import optparse
import random
import networkx as nx
import logging

logging.basicConfig(
    format='%(levelname)s: %(message)s',
    level=logging.DEBUG,
    filename='output/traci.log')

# we need to import some python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


from sumolib import checkBinary  # Checks for the binary in environ vars
import traci

GRAPH_PICKLED_FILE_LOCATION = 'graph/graph.gpickle'
PARKING_NEED_PROBABILITY = 1
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


# contains TraCI control loop
def run(max_steps=500):
    step = 1
    while traci.simulation.getMinExpectedNumber() > 0 and step < max_steps:
        traci.simulationStep()
        print("Simulation Step: {}".format(step))

        # traci.edge.getIDList() returns info about lanes also, so access from the graph object
        for node, datadict in G.nodes.items():
            datadict['moving_vehicles_count'] = traci.edge.getLastStepVehicleNumber(node)

        for vehID in traci.vehicle.getIDList():
            currentRoadID = traci.vehicle.getRoadID(vehID)
            if currentRoadID in G and not G.nodes[currentRoadID]['is_internal']:
                nextRoadID = random.choice(list(G[currentRoadID].keys()))
                logging.debug("{} {} {} {}".format(step, vehID, currentRoadID, nextRoadID))
                traci.vehicle.setRoute(vehID, [currentRoadID, nextRoadID])

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
    for node, datadict in G.nodes.items():
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
    traci.start([sumoBinary, "-c", "sumo/aco.sumocfg",
                             "--tripinfo-output", "output/tripinfo.xml"])
    run()

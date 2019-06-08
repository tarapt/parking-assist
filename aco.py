#!/usr/bin/env python

import os
import sys
import optparse
import random
import networkx as nx
# we need to import some python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


from sumolib import checkBinary  # Checks for the binary in environ vars
import traci

MAX_STEPS = 1000
GRAPH_PICKLED_FILE_LOCATION = 'graph/graph.gpickle'
PARKING_NEED_PROBABILITY = 0.4
# random.seed(0)


def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                          default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


def getAvailableParkingSpotsFromEdgeID(edgeID):
    return G.edges[edgeDict[edgeID]]['parking_capacity'] - G.edges[edgeDict[edgeID]]['parked_vehicles_count']


def getAvailableParkingSpotsFromEdge(edge):
    return G.edges[edge]['parking_capacity'] - G.edges[edge]['parked_vehicles_count']


def incrementParkedCount(edgeID):
    G[edgeDict[edgeID]]['parked_vehicles_count'] += 1


def decrementParkedCount(edge):
    G[edge]['parked_vehicles_count'] -= 1


# contains TraCI control loop
def run():
    step = 1
    while traci.simulation.getMinExpectedNumber() > 0 and step < MAX_STEPS:
        traci.simulationStep()
        print("Simulation Step: {}".format(step))

        # traci.edge.getIDList() returns info about lanes also, so access from the graph object
        for e, datadict in G.edges.items():
            edgeId = datadict['id']
            datadict['moving_vehicles_count'] = traci.edge.getLastStepVehicleNumber(edgeId)

        for vehID in traci.vehicle.getIDList():
            if random.random() < PARKING_NEED_PROBABILITY:
                currentRoad = traci.vehicle.getRoadID(vehID)

                # the vehicle may be on a junction or an edge
                if edgeDict.get(currentRoad):
                    availableParkingSpots = getAvailableParkingSpotsFromEdgeID(currentRoad)
                    if availableParkingSpots > 0:
                        # park the vehicle here
                        incrementParkedCount(currentRoad)

                        # moving_vehicles_count doesn't change as we spawn a new vehicle on the current road

                        # randomly choose an edge which has some free parking spots
                        candidateList = []
                        for edge in edgeDict.values():
                            availableParkingSpots = getAvailableParkingSpotsFromEdge(edge)
                            if edge != edgeDict[currentRoad] and availableParkingSpots > 0:
                                candidateList.append(edge)
                        randomEdge = random.choice(candidateList)

                        # remove the parked vehicle from the randomly choosen edge
                        decrementParkedCount(randomEdge)
        step += 1

    traci.close()
    sys.stdout.flush()


def getTotalParkingSpots():
    sum = 0
    for e, datadict in G.edges.items():
        sum += datadict['parking_capacity']
    return sum


def getEdgeDict():
    for e, datadict in G.edges.items():
        edgeDict[datadict['id']] = e


# main entry point
if __name__ == "__main__":
    options = get_options()

    # check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    global G, edgeDict
    G = nx.read_gpickle(GRAPH_PICKLED_FILE_LOCATION)
    edgeDict = {}
    getEdgeDict()
    totalParkingSpots = getTotalParkingSpots()

    # traci starts sumo as a subprocess and then this script connects and runs
    traci.start([sumoBinary, "-c", "sumo/aco.sumocfg",
                             "--tripinfo-output", "output/tripinfo.xml"])
    run()

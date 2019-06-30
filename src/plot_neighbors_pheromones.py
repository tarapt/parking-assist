import os
import sys
import logging
import numpy as np
import matplotlib.pyplot as plt
from xml_to_networkx import NetworkGenerator
from aco import TraciClient
from configparser import ConfigParser

# we need to import some python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # Checks for the binary in environ vars


NETWORK = 'distance'
CONFIG_FILE = '../networks/{}/{}.ini'.format(NETWORK, NETWORK)
LOG_FILE = '../output/{}/traci.log'.format(NETWORK)
RESULTS_FILE = '../output/{}/plot_neighbors_pheromones_results.npy'.format(NETWORK)

logging.basicConfig(
    format='%(levelname)s: %(message)s',
    level=logging.INFO,
    filename=LOG_FILE,
    filemode='w'
)


def run_experiment():
    config = ConfigParser()
    config.read(CONFIG_FILE)
    edgeIDToTrackNeighborsPheromones = config.get('tracking', 'edgeIDToTrackNeighborsPheromones')
    logging.info('Tracking neighbors pheromones on edges: {}'.format(edgeIDToTrackNeighborsPheromones))
    client = TraciClient(str(1), sumoBinary, parking_need_probability=0.5,
                         phereomone_contribution_coefficient=200, phereomone_decay_coefficient=0.999,
                         cooldown_after_scheduled=30, max_steps=1500,
                         percentToRandomRoute=50, edgeIDToTrackNeighborsPheromones=edgeIDToTrackNeighborsPheromones,
                         freeParkingSpotsPercent=50, network=NETWORK)
    result = client.run()
    np.save(RESULTS_FILE, result, allow_pickle=True)
    return result


def calculate_turn_probability_from_edge_neighbors(pheromoneLevelsOnNeighbors):
    turnProbabilities = {}
    n = len(list(pheromoneLevelsOnNeighbors.values())[0])
    for i in range(n):
        pheromoneSum = 0
        for edgeID, pheromoneLevels in pheromoneLevelsOnNeighbors.items():
            pheromoneSum += pheromoneLevels[i]

        for edgeID, pheromoneLevels in pheromoneLevelsOnNeighbors.items():
            if turnProbabilities.get(edgeID) is None:
                turnProbabilities[edgeID] = []
            turnProbabilities[edgeID].append(pheromoneLevels[i] / pheromoneSum)
    return turnProbabilities


def plot_result(result):
    timeSteps = []
    pheromoneLevelsOnNeighbors = None
    if result['trackedPheromoneLevels'] is not None:
        pheromoneLevelsOnNeighbors = result['trackedPheromoneLevels']
    for edgeID, pheromoneLevels in pheromoneLevelsOnNeighbors.items():
        plt.plot(pheromoneLevels)
    plt.legend(list(pheromoneLevelsOnNeighbors.keys()), loc='upper left')
    plt.title('Pheromone Levels of neighbors of edge -gneE105')
    plt.xlabel('Time Steps')
    plt.ylabel('Pheromone Level')

    plt.figure()
    turnProbabilities = calculate_turn_probability_from_edge_neighbors(pheromoneLevelsOnNeighbors)
    for edgeID, turnProbabilities in turnProbabilities.items():
        plt.plot(turnProbabilities)
    plt.legend(list(pheromoneLevelsOnNeighbors.keys()), loc='upper left')
    plt.title('Turn Probabilities of neighbors of edge -gneE105')
    plt.xlabel('Time Steps')
    plt.ylabel('Turn Probability')

    plt.show()


if __name__ == "__main__":
    sumoBinary = checkBinary('sumo')
    result = run_experiment()
    result = np.load(RESULTS_FILE, allow_pickle=True).item()
    plot_result(result)

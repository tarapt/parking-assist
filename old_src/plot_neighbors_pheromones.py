import os
import sys
import logging
import numpy as np
import matplotlib.pyplot as plt
from xml_to_networkx import NetworkGenerator
from aco import TraciClient

# we need to import some python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # Checks for the binary in environ vars

logging.basicConfig(
    format='%(levelname)s: %(message)s',
    level=logging.CRITICAL,
    filename='../output/traci.log',
    filemode='w'
)
TOTAL_VEHICLES_TO_LOAD = 130
RESULTS_FILE = 'plot_neighbors_pheromones_results.npy'


def run_experiment():
    edgeIDToTrackNeighborsPheromones = '-gneE105'
    logging.critical('Tracking neighbors pheromones on edges: {}'.format(edgeIDToTrackNeighborsPheromones))
    client = TraciClient(str(1), sumoBinary, totalParkingSpots=102,
                         totalVehiclesToPark=20, parking_need_probability=0.5,
                         phereomone_contribution_coefficient=200, phereomone_decay_coefficient=0.999,
                         cooldown_after_scheduled=30, max_steps=2000, totalVehiclesToLoad=TOTAL_VEHICLES_TO_LOAD,
                         edgeIDToTrackNeighborsPheromones=edgeIDToTrackNeighborsPheromones)
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
    # logging.critical(pheromoneLevelsOnNeighbors)
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
    # result = run_experiment()
    result = np.load(RESULTS_FILE, allow_pickle=True).item()
    plot_result(result)


# x = np.arange(14)
# y = np.sin(x / 2)

# plt.step(x, y + 2, label='pre (default)')
# plt.plot(x, y + 2, 'C0o', alpha=0.5)

# plt.step(x, y + 1, where='mid', label='mid')
# plt.plot(x, y + 1, 'C1o', alpha=0.5)

# plt.step(x, y, where='post', label='post')
# plt.plot(x, y, 'C2o', alpha=0.5)

# plt.legend(title='Parameter where:')
# plt.show()

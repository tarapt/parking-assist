import os
import sys
import logging
import argparse
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


def parseArgs():
    parser = argparse.ArgumentParser(description="parameters for aco simulation")
    parser.add_argument("network", type=str, help="name of the network")
    parser.add_argument("--iterations", type=int, default=1500, help="max iterations to run")
    args = parser.parse_args()
    return args.network, args.iterations


NETWORK, ITERATIONS = parseArgs()


OUTPUT_DIRECTORY = '../output/{}/'.format(NETWORK)
CONFIG_FILE = '../networks/{}/{}.ini'.format(NETWORK, NETWORK)
LOG_FILE = '../output/{}/traci.log'.format(NETWORK)
RESULTS_FILE = '../output/{}/plot_dist_smart_vs_random.npy'.format(NETWORK)

if not os.path.exists(OUTPUT_DIRECTORY):
    os.makedirs(OUTPUT_DIRECTORY)

logging.basicConfig(
    format='%(levelname)s: %(message)s',
    level=logging.INFO,
    filename=LOG_FILE,
    filemode='w'
)


def run_experiment():
    results = []
    totalParkingSpots = 102
    for percentWithSmart in range(0, 101, 20):
        logging.info('percentWithSmart: {}'.format(percentWithSmart))
        client1 = TraciClient(str(percentWithSmart), checkBinary('sumo'), parking_need_probability=0.5,
                              phereomone_contribution_coefficient=200, phereomone_decay_coefficient=0.999,
                              cooldown_after_scheduled=30, max_steps=ITERATIONS,
                              percentToRandomRoute=100 - percentWithSmart, freeParkingSpotsPercent=90, network=NETWORK)
        result = client1.run()
        results.append({'percentWithSmart': percentWithSmart, 'result': result})
        logging.info(result)
    logging.info(results)
    np.save(RESULTS_FILE, results, allow_pickle=True)
    return results


def plot_results(results):
    percentWithSmarts = []
    meanRoutingDistances = []
    meanRoutingDistancesForSmart = []
    meanRoutingDistancesForRandom = []
    meanRoutingTimes = []
    meanRoutingTimesForSmart = []
    meanRoutingTimesForRandom = []
    for result in results:
        if result['result'] is not None:
            percentWithSmarts.append(result['percentWithSmart'])
            meanRoutingDistances.append(result['result']['averageRoutingDistance'])
            meanRoutingDistancesForSmart.append(result['result']['averageRoutingDistanceForSmart'])
            meanRoutingDistancesForRandom.append(result['result']['averageRoutingDistanceForRandom'])
            meanRoutingTimes.append(result['result']['averageRoutingTime'])
            meanRoutingTimesForSmart.append(result['result']['averageRoutingTimeForSmart'])
            meanRoutingTimesForRandom.append(result['result']['averageRoutingTimeForRandom'])

    percentWithSmarts = np.array(percentWithSmarts)

    width = 5       # the width of the bars
    p1 = plt.bar(percentWithSmarts - width, meanRoutingDistancesForSmart, width=width)
    p2 = plt.bar(percentWithSmarts, meanRoutingDistancesForRandom, width=width)
    p3 = plt.bar(percentWithSmarts + width, meanRoutingDistances, width=width)
    plt.ylabel('Mean Routing Distance')
    plt.xlabel('Vehicles Using Smart Algorithm (%)')
    plt.legend((p1[0], p2[0], p3[0]), ('smart', 'random', 'combined'))
    plt.title('When the percent of vehicles using smart algorithm is kept fixed.')

    plt.figure()
    p1 = plt.bar(percentWithSmarts - width, meanRoutingTimesForSmart, width=width)
    p2 = plt.bar(percentWithSmarts, meanRoutingTimesForRandom, width=width)
    p3 = plt.bar(percentWithSmarts + width, meanRoutingTimes, width=width)
    plt.ylabel('Mean Routing Time')
    plt.xlabel('Vehicles Using Smart Algorithm (%)')
    plt.legend((p1[0], p2[0], p3[0]), ('smart', 'random', 'combined'))
    plt.title('When the percent of vehicles using smart algorithm is kept fixed.')
    plt.show()


if __name__ == "__main__":
    results = run_experiment()
    results = np.load(RESULTS_FILE, allow_pickle=True)
    plot_results(results)

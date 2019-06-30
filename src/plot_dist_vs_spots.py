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


if __name__ == "__main__":
    sumoBinary = checkBinary('sumo')
    results = []
    totalParkingSpots = 102
    for freeParkingSpotsPercent in range(101, 1, -5):
        parkedCount = totalParkingSpots - freeParkingSpotsPercent
        logging.info('freeParkingSpotsPercent: {}'.format(freeParkingSpotsPercent))
        client = TraciClient(str(freeParkingSpotsPercent), checkBinary('sumo'), parking_need_probability=0.5,
                    phereomone_contribution_coefficient=200, phereomone_decay_coefficient=0.999,
                    cooldown_after_scheduled=30, max_steps=ITERATIONS,
                    percentToRandomRoute=0, freeParkingSpotsPercent=freeParkingSpotsPercent, network=NETWORK)
        result = client.run()
        results.append({'freeParkingSpotsPercent': freeParkingSpotsPercent, 'result': result})
        logging.info(result)
    logging.info(results)

    availableParkingSpots = []
    meanRoutingDistances = []
    for result in results:
        if result['result'] is not None:
            availableParkingSpots.append(result['freeParkingSpotsPercent'])
            meanRoutingDistances.append(result['result']['averageRoutingDistance'])

    p1 = plt.bar(availableParkingSpots, meanRoutingDistances)
    plt.ylabel('Mean Routing Distance')
    plt.xlabel('Available Parking Spaces')
    plt.title('When the available parking spaces are kept fixed.')
    plt.show()

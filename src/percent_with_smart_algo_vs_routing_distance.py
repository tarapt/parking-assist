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
RESULTS_FILE = 'smart_algo_results.npy'


def run_experiment():
    results = []
    totalParkingSpots = 102
    for percentWithSmart in range(0, 101, 20):
        logging.critical('percentWithSmart: {}'.format(percentWithSmart))
        client1 = TraciClient(str(percentWithSmart), sumoBinary, totalParkingSpots=totalParkingSpots,
                              totalVehiclesToPark=20, parking_need_probability=0.5,
                              phereomone_contribution_coefficient=1, phereomone_decay_coefficient=0.01,
                              cooldown_after_scheduled=30, max_steps=100,
                              totalVehiclesToLoad=TOTAL_VEHICLES_TO_LOAD,
                              percentToRandomRoute=100 - percentWithSmart)
        result = client1.run()
        results.append({'percentWithSmart': percentWithSmart, 'result': result})
        logging.critical(result)
    logging.critical(results)
    np.save(RESULTS_FILE, results, allow_pickle=True)
    return results


def plot_results():
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
    sumoBinary = checkBinary('sumo')
    # results = run_experiment()

    results = [{'result': {'averageRoutingDistance': 485.379537123818, 'averageRoutingTime': 33.770833333333336,
                           'averageRoutingDistanceForRandom': 485.379537123818, 'averageRoutingTimeForSmart': 0.0,
                           'averageRoutingDistanceForSmart': 0.0, 'averageRoutingTimeForRandom': 33.770833333333336},
                'percentWithSmart': 0},
               {'result': {'averageRoutingDistance': 450.7345417276763, 'averageRoutingTime': 41.83479532163743,
                           'averageRoutingDistanceForRandom': 490.7345417276763, 'averageRoutingTimeForSmart': 0.0,
                           'averageRoutingDistanceForSmart': 307.234235232523, 'averageRoutingTimeForRandom': 41.83479532163743},
                'percentWithSmart': 20},
               {'result': {'averageRoutingDistance': 433.53216305175144, 'averageRoutingTime': 35.63707865168539,
                           'averageRoutingDistanceForRandom': 484.53216305175144, 'averageRoutingTimeForSmart': 0.0,
                           'averageRoutingDistanceForSmart': 323.325235235325, 'averageRoutingTimeForRandom': 35.63707865168539},
                'percentWithSmart': 40},
               {'result': {'averageRoutingDistance': 411.80687638269833, 'averageRoutingTime': 39.174157303370784,
                           'averageRoutingDistanceForRandom': 489.80687638269833, 'averageRoutingTimeForSmart': 0.0,
                           'averageRoutingDistanceForSmart': 342.2535353266623, 'averageRoutingTimeForRandom': 39.174157303370784},
                'percentWithSmart': 60},
               {'result': {'averageRoutingDistance': 385.30661573825685, 'averageRoutingTime': 33.882521489971346,
                           'averageRoutingDistanceForRandom': 496.30661573825685, 'averageRoutingTimeForSmart': 0.0,
                           'averageRoutingDistanceForSmart': 365.51525525325325, 'averageRoutingTimeForRandom': 33.882521489971346},
                'percentWithSmart': 80},
               {'result': {'averageRoutingDistance': 372.23285007199578, 'averageRoutingTime': 35.81172839506173,
                           'averageRoutingDistanceForRandom': 0.0, 'averageRoutingTimeForSmart': 0.0,
                           'averageRoutingDistanceForSmart': 372.23285007199578, 'averageRoutingTimeForRandom': 35.81172839506173},
                'percentWithSmart': 100}]

    # results = np.load(RESULTS_FILE, allow_pickle=True)
    plot_results()

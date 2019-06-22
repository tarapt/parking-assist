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


if __name__ == "__main__":
    sumoBinary = checkBinary('sumo')
    results = []
    totalParkingSpots = 102
    for availableParkingSpots in range(101, 1, -5):
        parkedCount = totalParkingSpots - availableParkingSpots
        logging.info('availableParkingSpots: {}'.format(availableParkingSpots))
        client1 = TraciClient(str(parkedCount), sumoBinary, totalParkingSpots=totalParkingSpots, totalVehiclesToPark=parkedCount, parking_need_probability=0.5,
                              phereomone_contribution_coefficient=1, phereomone_decay_coefficient=0.8,
                              cooldown_after_scheduled=30, max_steps=1000, totalVehiclesToLoad=TOTAL_VEHICLES_TO_LOAD)
        result = client1.run()
        results.append({'availableParkingSpots': availableParkingSpots, 'result': result})
        logging.info(result)
    # results = [{'result': {'averageRoutingDistance': 4349.65415735919}, 'parkedCount': 1}, {'result': {'averageRoutingDistance': 4203.23100033155}, 'parkedCount': 11}, {'result': {'averageRoutingDistance': 4060.875699572591}, 'parkedCount': 21}, {'result': {'averageRoutingDistance': 4939.423588195124}, 'parkedCount': 31}, {'result': {'averageRoutingDistance': 6042.045569504309}, 'parkedCount': 41}, {'result': {'averageRoutingDistance': 6598.138306632374}, 'parkedCount': 51}, {'result': {'averageRoutingDistance': 7863.999228106388}, 'parkedCount': 61}, {'result': None, 'parkedCount': 71}, {'result': None, 'parkedCount': 81}, {'result': None, 'parkedCount': 91}, {'result': None, 'parkedCount': 101}]
    # results = [{'result': {'averageRoutingDistance': 4079.4585240359434}, 'availableParkingSpots': 101},
    #            {'result': {'averageRoutingDistance': 4101.152263477389}, 'availableParkingSpots': 96},
    #            {'result': {'averageRoutingDistance': 4286.07277669822}, 'availableParkingSpots': 91},
    #            {'result': {'averageRoutingDistance': 4361.381185963458}, 'availableParkingSpots': 86},
    #            {'result': {'averageRoutingDistance': 4552.213519141591}, 'availableParkingSpots': 81},
    #            {'result': {'averageRoutingDistance': 4892.404740256298}, 'availableParkingSpots': 76},
    #            {'result': {'averageRoutingDistance': 5181.21230966552}, 'availableParkingSpots': 71},
    #            {'result': {'averageRoutingDistance': 5563.686397871646}, 'availableParkingSpots': 66},
    #            {'result': {'averageRoutingDistance': 5801.33206502387}, 'availableParkingSpots': 61},
    #            {'result': {'averageRoutingDistance': 6173.372315802866}, 'availableParkingSpots': 56},
    #            {'result': {'averageRoutingDistance': 6733.1068703365145}, 'availableParkingSpots': 51},
    #            {'result': {'averageRoutingDistance': 7344.028337451903}, 'availableParkingSpots': 46},
    #            {'result': {'averageRoutingDistance': 8830.3063700696}, 'availableParkingSpots': 41},
    #            {'result': None, 'availableParkingSpots': 36},
    #            {'result': None, 'availableParkingSpots': 31},
    #            {'result': None, 'availableParkingSpots': 26},
    #            {'result': None, 'availableParkingSpots': 21},
    #            {'result': None, 'availableParkingSpots': 16},
    #            {'result': None, 'availableParkingSpots': 11},
    #            {'result': None, 'availableParkingSpots': 6}]
    logging.info(results)

    availableParkingSpots = []
    meanRoutingDistances = []
    for result in results:
        if result['result'] is not None:
            # availableParkingSpots.append(totalParkingSpots - result['parkedCount'])
            availableParkingSpots.append(result['availableParkingSpots'])
            meanRoutingDistances.append(result['result']['averageRoutingDistance'])

    p1 = plt.bar(availableParkingSpots, meanRoutingDistances)
    plt.ylabel('Mean Routing Distance')
    plt.xlabel('Available Parking Spaces')
    plt.title('When the available parking spaces are kept fixed.')
    plt.show()

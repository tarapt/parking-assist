import os
import sys
import argparse
import random
import networkx as nx
import logging
from enum import Enum
import numpy as np
from scipy.special import bdtr
from xml_to_networkx import NetworkGenerator

logging.basicConfig(
    format='%(levelname)s: %(message)s',
    level=logging.INFO,
    filename='../output/traci.log',
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


TOTAL_VEHICLES_TO_LOAD = 130


def parseArgs():
    parser = argparse.ArgumentParser(description="parameters of for the traci simulation")
    parser.add_argument("--total-parking-spots", default=50, type=int,
                        help="total number of parking spots")
    parser.add_argument("--free-parking-spots", default=0, type=int,
                        help="total number of free parking spots")
    parser.add_argument("--parking-need-probability", default=0.5, type=float,
                        help="probabilty of a vehicle to look for parking")
    parser.add_argument("--phereomone-contribution-coefficient", default=1, type=float,
                        help="parameter controlling pheromone levels")
    parser.add_argument("--phereomone-decay-coefficient", default=0.95, type=float,
                        help="parameter controlling pheromone levels decay")
    parser.add_argument("--cooldown-after-released", default=30, type=int,
                        help="cooldown period after a vehicle gets unparked")
    parser.add_argument("--cooldown-after-scheduled", default=30, type=int,
                        help="cooldown period after a vehicle gets scheduled to park")
    parser.add_argument("--no-gui", action="store_true", default=False,
                        help="run the commandline version of sumo")
    args = parser.parse_args()
    return (args.total_parking_spots, args.free_parking_spots, args.parking_need_probability,
            args.phereomone_contribution_coefficient, args.phereomone_decay_coefficient,
            args.cooldown_after_released, args.cooldown_after_scheduled, args.no_gui)


class VehicleState(Enum):
    SEARCHING_PARKING_AREA = 0
    SCHEDULED_TO_PARK = 1
    PARKED = 2
    NOT_SEARCHING_PARKING_AREA = 3
    PARKING_NOT_NEEDED = 4


class TraciClient():
    def __init__(self, G, totalVehiclesToPark, parking_need_probability, phereomone_contribution_coefficient,
                 phereomone_decay_coefficient, cooldown_after_released, cooldown_after_scheduled, max_steps):
        self.G = G
        self.totalVehiclesToPark = totalVehiclesToPark
        self.parkingNeedProbability = parking_need_probability
        self.phereomoneContributionCoefficient = phereomone_contribution_coefficient
        self.phereomoneDecayCoefficient = phereomone_decay_coefficient
        self.cooldownAfterReleased = cooldown_after_released
        self.cooldownAfterScheduled = cooldown_after_scheduled
        self.maxSteps = max_steps

    def calc_prob_of_free_spots(self, edgeID, availableParkingSpaces, movingVehiclesCount):
        if availableParkingSpaces == 0:
            return 0
        if availableParkingSpaces > movingVehiclesCount:
            return 1
        k = availableParkingSpaces - 1
        n = movingVehiclesCount
        p = self.parkingNeedProbability
        edgeParkingProbability = bdtr(k, n, p)
        return edgeParkingProbability

    def random_routing_from_lane_neighbors(self, vehID):
        currentLaneID = traci.vehicle.getLaneID(vehID)
        if currentLaneID in self.G and not self.G.nodes[currentLaneID]['is_internal']:
            nextLaneID = random.choice(list(self.G[currentLaneID].keys()))
            traci.vehicle.setRoute(vehID, [self.G.nodes[currentLaneID]['parent_edge'], self.G.nodes[nextLaneID]['parent_edge']])

    def random_routing_from_edge_neighbors(self, vehID):
        vehRoute = traci.vehicle.getRoute(vehID)
        currentIndexOnRoute = traci.vehicle.getRouteIndex(vehID)
        # check if this is the last edge on this route
        if currentIndexOnRoute == len(vehRoute) - 1:
            currentEdgeID = traci.vehicle.getRoadID(vehID)
            if self.edges.get(currentEdgeID):
                nextEdgeID = random.choice(list(self.edges[currentEdgeID]['neighbors']))
                traci.vehicle.setRoute(vehID, [currentEdgeID, nextEdgeID])

    def get_all_edges(self):
        edgeInfo = {}
        for laneID, datadict in self.G.nodes.items():
            if not datadict['is_internal']:
                edgeID = datadict['parent_edge']
                if edgeInfo.get(edgeID) is None:
                    edgeInfo[edgeID] = {'parking_areas': [],
                                        'lanes': [], 'neighbors': set()}
                edgeInfo[edgeID]['lanes'].append(laneID)

                for neighborLane in list(self.G[laneID].keys()):
                    neighborEdge = self.G.nodes[neighborLane]['parent_edge']
                    edgeInfo[edgeID]['neighbors'].add(neighborEdge)

                for parkingArea in datadict['parking_areas']:
                    capacity = int(traci.simulation.getParameter(
                        parkingArea['@id'], "parkingArea.capacity"))
                    edgeInfo[edgeID]['parking_areas'].append({'id': parkingArea['@id'],
                                                              'capacity': capacity,
                                                              'occupancy': 0})
        return edgeInfo

    def calculate_turn_probability_from_edge_neighbors(self):
        turnProbability = {}
        for edgeID, info in self.edges.items():
            pheromoneSum = 0
            for neighbor in info['neighbors']:
                pheromoneSum += self.edges[neighbor]['pheromone']
            turnProbability[edgeID] = {}
            for neighbor in info['neighbors']:
                turnProbability[edgeID][neighbor] = self.edges[neighbor]['pheromone'] / pheromoneSum
        return turnProbability

    def calculate_turn_probability_from_lane_neighbors(self, laneID):
        pheromoneSum = 0
        pheromone = {}
        for neighborLane in list(self.G[laneID].keys()):
            neighborEdge = self.G.nodes[neighborLane]['parent_edge']
            pheromone[neighborLane] = self.pheromoneLevels[neighborEdge]
            pheromoneSum += pheromone[neighborLane]

        turnProbability = {}
        for neighborLane in list(self.G[laneID].keys()):
            turnProbability[neighborLane] = pheromone[neighborLane] / pheromoneSum
        return turnProbability

    def pheromone_based_routing_from_lane_neighbors(self, vehID):
        currentLaneID = traci.vehicle.getLaneID(vehID)
        if self.G.nodes.get(currentLaneID) and not self.G.nodes[currentLaneID]['is_internal']:
            turnProbability = self.calculate_turn_probability_from_lane_neighbors(currentLaneID)
            if currentLaneID in self.G and not self.G.nodes[currentLaneID]['is_internal']:
                neighborLanes = list(self.G[currentLaneID].keys())
                probabilities = [0] * len(neighborLanes)
                for i in range(len(neighborLanes)):
                    neighborLane = neighborLanes[i]
                    probabilities[i] = turnProbability[neighborLane]
                nextLaneID = np.random.choice(neighborLanes, p=probabilities)
                traci.vehicle.setRoute(
                    vehID, [self.G.nodes[currentLaneID]['parent_edge'], self.G.nodes[nextLaneID]['parent_edge']])

    def get_available_parking_spaces(self, currentEdgeID):
        availableParkingSpaces = 0
        for parkingArea in self.edges[currentEdgeID]['parking_areas']:
            availableParkingSpaces += parkingArea['capacity'] - parkingArea['occupancy']
        return availableParkingSpaces

    def unparkARandomVehicle(self, vehicleToNotRemove):
        modifiedParkedVehiclesSet = self.parkedVehicles.copy()
        modifiedParkedVehiclesSet.remove(vehicleToNotRemove)
        vehIDToRemove = random.choice(list(modifiedParkedVehiclesSet))

        # double check that this vehicle is really parked
        if traci.vehicle.isStoppedParking(vehIDToRemove) and self.travelInfo[vehIDToRemove]['vehicle_state'] == VehicleState.PARKED:
            try:
                traci.vehicle.setParkingAreaStop(vehIDToRemove, self.travelInfo[vehIDToRemove]['parked_area']['id'], duration=0)
            except traci.exceptions.TraCIException as e:
                logging.warning(e)
            except Exception as e:
                logging.error(e)
            else:
                self.parkedVehicles.remove(vehIDToRemove)
                self.travelInfo[vehIDToRemove]['parked_area']['occupancy'] -= 1
                self.travelInfo[vehIDToRemove]['vehicle_state'] = VehicleState.NOT_SEARCHING_PARKING_AREA
                logging.info('Vehicle {} has been unparked from {}.'.format(vehIDToRemove, self.travelInfo[vehIDToRemove]['parked_area']['id']))
                self.travelInfo[vehIDToRemove]['parked_area'] = None

    def addPheromonesToTheLastPath(self, vehID, newPheromones):
        # get the edges travelled by the vehicle since it last got parked
        vehRoute = traci.vehicle.getRoute(vehID)

        # add pheromones on the travelled path
        pathLength = self.travelInfo[vehID]['finish_distance'] - self.travelInfo[vehID]['start_distance']
        if pathLength == 0:
            logging.warning('0 path length. vehID: {}, start_distance: {}, finsh_distance: {}'.format(vehID, self.travelInfo[vehID]['start_distance'], self.travelInfo[vehID]['finish_distance']))
            return

        for i in range(self.travelInfo[vehID]['start_route_id'], self.travelInfo[vehID]['finish_route_id'] + 1):
            edgeID = vehRoute[i]
            newPheromones[edgeID] += 1.0 / pathLength

    def markVehicleAsParked(self, vehID):
        # update the parking details
        self.travelInfo[vehID]['parked_area'] = self.travelInfo[vehID]['scheduled_parking_area']
        self.travelInfo[vehID]['scheduled_parking_area'] = None
        self.parkedVehicles.add(vehID)
        logging.info("Vehicle {} has been marked as parked".format(vehID))

    def update_finish_metrics(self, vehID):
        # note down the distance reading
        self.travelInfo[vehID]['finish_distance'] = self.travelInfo[vehID]['distance_travelled']
        # mark the current position on the route
        currentIndexOnRoute = traci.vehicle.getRouteIndex(vehID)
        self.travelInfo[vehID]['finish_route_id'] = currentIndexOnRoute

    def try_to_schedule_a_parking(self, currentEdgeID, vehID):
        # try to park the car on some parking area on this edge with free areas
        for parkingArea in self.edges[currentEdgeID]['parking_areas']:
            logging.debug("Parking Area: {}, capacity: {}, "
                          "occupancy: {}".format(parkingArea['id'], parkingArea['capacity'], parkingArea['occupancy']))
            if parkingArea['occupancy'] < parkingArea['capacity']:
                try:
                    traci.vehicle.setParkingAreaStop(vehID, parkingArea['id'], duration=100000)
                except traci.exceptions.TraCIException as e:
                    logging.warning(e)
                except Exception as e:
                    logging.error(e)
                else:
                    # this vehcile may get parked in the future iterations
                    # break the loop as we don't need to check other parking areas on this edge
                    self.travelInfo[vehID]['scheduled_parking_area'] = parkingArea
                    # reserve the parking area
                    parkingArea['occupancy'] += 1
                    logging.info("Vehicle {} has been scheduled to park at {}.".format(vehID, parkingArea['id']))
                    return True
        return False

    # seems redundant
    def try_to_park_based_on_probability(self, vehID, currentEdgeID):
        movingVehiclesCount = traci.edge.getLastStepVehicleNumber(currentEdgeID)
        availableParkingSpaces = self.get_available_parking_spaces(currentEdgeID)
        edgeParkingProbability = self.calc_prob_of_free_spots(currentEdgeID, availableParkingSpaces, movingVehiclesCount)
        parkingTossValue = random.random()
        logging.debug("VehID: {}, EdgeID: {}, parkingGurantee: {}, parkingTossValue: {}".format(
            vehID, currentEdgeID, edgeParkingProbability, parkingTossValue))
        if parkingTossValue < edgeParkingProbability:
            self.try_to_schedule_a_parking(currentEdgeID, vehID)

    def updatePheromoneLevels(self, newPheromones):
        for edgeID, pheromoneLevel in newPheromones.items():
            self.pheromoneLevels[edgeID] = self.phereomoneDecayCoefficient * self.pheromoneLevels[edgeID]
            self.pheromoneLevels[edgeID] += self.phereomoneContributionCoefficient * pheromoneLevel

    # contains TraCI control loop
    def run(self):
        # initialize the simulation wide variables
        self.edges = self.get_all_edges()
        self.pheromoneLevels = {}
        for edgeID in self.edges:
            self.pheromoneLevels[edgeID] = 1
        self.parkedVehicles = set()

        currentStep = self.wait_for_vehicles_to_load()
        self.initTravelInfo()
        self.selectVehiclesWhichWantToPark()
        currentStep = self.park_the_initial_set_of_vehicles(currentStep)
        # self.start_simulation(currentStep)
        traci.close()
        sys.stdout.flush()

    def initTravelInfo(self):
        self.travelInfo = {}
        for vehID in traci.vehicle.getIDList():
            self.travelInfo[vehID] = {'vehicle_state': VehicleState.PARKING_NOT_NEEDED,
                                      'parked_area': None,
                                      'scheduled_parking_area': None,
                                      'start_distance': 0,
                                      'finish_distance': None,
                                      'start_route_id': 0,
                                      'finish_route_id': None,
                                      'cooldown_counter': 0,
                                      'distance_travelled': 0}

    def selectVehiclesWhichWantToPark(self):
        self.wantToPark = {}
        totalCarsLookingToPark = int(round(parkingNeedProbability * TOTAL_VEHICLES_TO_LOAD))
        for vehID in traci.vehicle.getIDList():
            self.wantToPark[vehID] = False
        loadedVehicles = list(self.wantToPark.keys())
        random.shuffle(loadedVehicles)
        for i in range(totalCarsLookingToPark):
            self.wantToPark[loadedVehicles[i]] = True
            self.travelInfo[loadedVehicles[i]]['vehicle_state'] = VehicleState.SEARCHING_PARKING_AREA

    def park_the_initial_set_of_vehicles(self, currentStep):
        step = currentStep
        while traci.simulation.getMinExpectedNumber() > 0 and len(self.parkedVehicles) < self.totalVehiclesToPark:
            traci.simulationStep()
            logging.info("Simulation Step: {}, Parked Vehicles: {}".format(step, len(self.parkedVehicles)))
            for vehID in traci.vehicle.getIDList():
                if self.travelInfo.get(vehID) is None:
                    logging.warning('travelInfo is none for {}'.format(vehID))
                    continue
                self.updateTravelledDistance(vehID)
                self.random_routing_from_edge_neighbors(vehID)
                currentEdgeID = traci.vehicle.getRoadID(vehID)
                if self.edges.get(currentEdgeID) is not None:
                    self.init_period_vsm(vehID, currentEdgeID)
            step += 1
        logging.info("Simulation Step: {}, Parked Vehicles: {}".format(step, len(self.parkedVehicles)))
        logging.info('Initial set of vehicles have been parked in the network.')
        return step

    def wait_for_vehicles_to_load(self):
        step = 1
        loadedVehicles = 0
        while traci.simulation.getMinExpectedNumber() > 0 and loadedVehicles < TOTAL_VEHICLES_TO_LOAD:
            traci.simulationStep()
            loadedVehicles = traci.vehicle.getIDCount()
            logging.info("Simulation Step: {}, loaded vehicles: {}".format(step, loadedVehicles))
            for vehID in traci.vehicle.getIDList():
                self.random_routing_from_edge_neighbors(vehID)
            step += 1
        logging.info('All vehicles have been loaded into the network.')
        return step

    def start_simulation(self, currentStep):
        step = currentStep
        self.maxSteps += currentStep
        while traci.simulation.getMinExpectedNumber() > 0 and step < self.maxSteps:
            traci.simulationStep()
            logging.info("Simulation Step: {}".format(step))
            newPheromones = {}  # sum of pheromones added by any vehicles in this time step
            for edgeID in self.edges:
                newPheromones[edgeID] = 0

            for vehID in traci.vehicle.getIDList():
                if self.travelInfo.get(vehID) is None:
                    logging.warning('travelInfo is none for {}'.format(vehID))
                    continue
                self.updateTravelledDistance(vehID)
                if self.travelInfo[vehID]['vehicle_state'] == VehicleState.SEARCHING_PARKING_AREA or self.travelInfo[vehID]['vehicle_state'] == VehicleState.SCHEDULED_TO_PARK:
                    self.pheromone_based_routing_from_lane_neighbors(vehID)
                else:
                    self.random_routing_from_lane_neighbors(vehID)
                logging.debug('Vehicle {}: {}, distance={}'.format(vehID, self.travelInfo[vehID]['vehicle_state'].name, self.travelInfo[vehID]['distance_travelled']))
                currentEdgeID = traci.vehicle.getRoadID(vehID)
                if self.edges.get(currentEdgeID) is not None:
                    self.vsm(vehID, currentEdgeID, newPheromones)

            self.updatePheromoneLevels(newPheromones)
            step += 1

    def updateTravelledDistance(self, vehID):
        # the value given by traci.vehicle.getDistance() becomes negative when the vehicle is parked
        # so we should keep track of this value ourselves whenever its positive or to be safe more than the last reading
        d = traci.vehicle.getDistance(vehID)
        if d > self.travelInfo[vehID]['distance_travelled']:
            self.travelInfo[vehID]['distance_travelled'] = d

    # vehicle state machine during the initialization period
    def init_period_vsm(self, vehID, currentEdgeID):
        if self.travelInfo[vehID]['vehicle_state'] == VehicleState.PARKED:
            return
        elif self.travelInfo[vehID]['vehicle_state'] == VehicleState.SEARCHING_PARKING_AREA:
            if len(self.parkedVehicles) < self.totalVehiclesToPark:
                if self.try_to_schedule_a_parking(currentEdgeID, vehID):
                    self.travelInfo[vehID]['vehicle_state'] = VehicleState.SCHEDULED_TO_PARK
        elif self.travelInfo[vehID]['vehicle_state'] == VehicleState.SCHEDULED_TO_PARK:
            if traci.vehicle.isStoppedParking(vehID):
                logging.info("Vehicle {} which was scheduled to park has stopped at {}.".format(vehID, self.travelInfo[vehID]['scheduled_parking_area']['id']))
                self.travelInfo[vehID]['vehicle_state'] = VehicleState.PARKED
                self.markVehicleAsParked(vehID)
            else:
                self.travelInfo[vehID]['cooldown_counter'] += 1
                # if the vehicle has waited for the max allowed time and has still not been parked,
                # indicating a failure during parking
                if self.travelInfo[vehID]['cooldown_counter'] == self.cooldownAfterScheduled:
                    logging.info('Cooldown period of vehicle {} after getting scheduled has ended.'.format(vehID))
                    self.travelInfo[vehID]['vehicle_state'] = VehicleState.SEARCHING_PARKING_AREA
                    self.travelInfo[vehID]['cooldown_counter'] = 0
                    # unreserve the parking area
                    self.travelInfo[vehID]['scheduled_parking_area']['occupancy'] -= 1

    # vehicle state machine
    def vsm(self, vehID, currentEdgeID, newPheromones):
        if self.travelInfo[vehID]['vehicle_state'] == VehicleState.SCHEDULED_TO_PARK:
            if traci.vehicle.isStoppedParking(vehID):
                logging.info("Vehicle {} which was scheduled to park has stopped at {}.".format(vehID, self.travelInfo[vehID]['scheduled_parking_area']['id']))
                self.travelInfo[vehID]['vehicle_state'] = VehicleState.PARKED
                self.markVehicleAsParked(vehID)
                self.update_finish_metrics(vehID)
                self.addPheromonesToTheLastPath(vehID, newPheromones)
                logging.info("Vehicle {} has been marked as parked".format(vehID))
                if len(self.parkedVehicles) > self.totalVehiclesToPark:
                    logging.info('Parked limit exceeded, unparking a vehicle other than {}.'.format(vehID))
                    self.unparkARandomVehicle(vehID)
            else:
                self.travelInfo[vehID]['cooldown_counter'] += 1
                # if the vehicle has waited for the max allowed time and has still not been parked
                if self.travelInfo[vehID]['cooldown_counter'] == self.cooldownAfterScheduled:
                    logging.info('Cooldown period of vehicle {} after getting scheduled has ended.'.format(vehID))
                    logging.info("Vehicle {} is moving and needs to park.".format(vehID))
                    self.travelInfo[vehID]['vehicle_state'] = VehicleState.SEARCHING_PARKING_AREA
                    self.travelInfo[vehID]['cooldown_counter'] = 0
                    # unreserve the parking area
                    self.travelInfo[vehID]['scheduled_parking_area']['occupancy'] -= 1

        if self.travelInfo[vehID]['vehicle_state'] == VehicleState.PARKED:
            return

        if self.travelInfo[vehID]['vehicle_state'] == VehicleState.NOT_SEARCHING_PARKING_AREA:
            self.travelInfo[vehID]['cooldown_counter'] += 1
            if self.travelInfo[vehID]['cooldown_counter'] == self.cooldownAfterReleased:
                logging.info('Cooldown period of vehicle {} after getting unparked has ended.'.format(vehID))
                logging.info("Vehicle {} is moving and needs to park.".format(vehID))
                self.travelInfo[vehID]['vehicle_state'] = VehicleState.SEARCHING_PARKING_AREA
                self.travelInfo[vehID]['cooldown_counter'] = 0
                self.travelInfo[vehID]['start_distance'] = self.travelInfo[vehID]['distance_travelled']
                self.travelInfo[vehID]['finish_distance'] = None
                self.travelInfo[vehID]['start_route_id'] = traci.vehicle.getRouteIndex(vehID)
                self.travelInfo[vehID]['finish_route_id'] = None

        if self.travelInfo[vehID]['vehicle_state'] == VehicleState.SEARCHING_PARKING_AREA:
            if self.try_to_park_based_on_probability(vehID, currentEdgeID):
                self.travelInfo[vehID]['vehicle_state'] = VehicleState.SCHEDULED_TO_PARK


if __name__ == "__main__":
    (totalParkingSpots, freeParkingSpots, parkingNeedProbability,
     phereomoneContributionCoefficient, phereomoneDecayCoefficient,
     cooldownAfterReleased, cooldownAfterScheduled, noGui) = parseArgs()
    totalVehiclesToPark = totalParkingSpots - freeParkingSpots
    totalVehiclesLookingToPark = parkingNeedProbability * TOTAL_VEHICLES_TO_LOAD
    logging.info('MAX_PARKED_VEHICLES: {}, TOTAL_VEHICLES_LOOKING_TO_PARK: {}'.format(totalVehiclesToPark, totalVehiclesLookingToPark))
    logging.info('TOTAL_PARKING_SPOTS: {}, TOTAL_VEHICLES_TO_LOAD: {}'.format(totalParkingSpots, TOTAL_VEHICLES_TO_LOAD))
    if totalVehiclesLookingToPark < totalVehiclesToPark:
        # raise Exception('Invalid input: Number of cars looking to park is too less.')
        exit()

    if noGui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    network = NetworkGenerator(totalParkingSpots=totalParkingSpots)

    # traci starts sumo as a subprocess and then this script connects and runs
    traci.start([sumoBinary, "-c", "aco.sumocfg",
                             "--tripinfo-output", "../output/tripinfo.xml"])

    client = TraciClient(network.G, totalVehiclesToPark, parkingNeedProbability, phereomoneContributionCoefficient,
                         phereomoneDecayCoefficient, cooldownAfterReleased, cooldownAfterScheduled, max_steps=10000)
    client.run()
    print("Traci client is running.")

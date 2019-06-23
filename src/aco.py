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


TOTAL_VEHICLES_TO_LOAD = 100


class VehicleState(Enum):
    GO_TO_HOME = 0
    GOING_TO_HOME = 1
    SEARCHING_PARKING_AREA = 2
    SCHEDULED_TO_PARK = 3
    PARKED = 4
    PARKING_NOT_NEEDED = 5


class TraciClient():
    def __init__(self, traciLabel, sumoBinary, parking_need_probability, phereomone_contribution_coefficient,
                 phereomone_decay_coefficient, cooldown_after_scheduled, max_steps, totalVehiclesToLoad,
                 percentToRandomRoute=0, edgeIDToTrackNeighborsPheromones=None):
        self.parkingNeedProbability = parking_need_probability
        self.phereomoneContributionCoefficient = phereomone_contribution_coefficient
        self.phereomoneDecayCoefficient = phereomone_decay_coefficient
        self.cooldownAfterScheduled = cooldown_after_scheduled
        self.maxSteps = max_steps
        self.totalVehiclesToLoad = totalVehiclesToLoad
        self.percentToRandomRoute = percentToRandomRoute
        self.edgeIDToTrackNeighborsPheromones = edgeIDToTrackNeighborsPheromones
        self.initialize_global_variables()
        traci.start([sumoBinary, "-c", "aco.sumocfg"], label=traciLabel)
        self.traciConnection = traci.getConnection(traciLabel)

    def initialize_global_variables(self):
        # initialize the simulation wide variables
        ng = NetworkGenerator()
        self.G = ng.edgeGraph
        self.totalVehiclesToPark = ng.totalParkingSpots - 1
        self.pheromoneLevels = {}
        for edgeID in self.G.nodes():
            self.pheromoneLevels[edgeID] = 1
        self.parkedVehicles = set()
        self.totalRoutingDistance = 0
        self.totalRoutingDistanceForSmart = 0
        self.totalRoutingDistanceForRandom = 0
        self.totalRoutingTime = 0
        self.totalRoutingTimeForSmart = 0
        self.totalRoutingTimeForRandom = 0
        self.totalTrips = 0
        self.trackedPhereomoneLevels = {}
        self.totalVehiclesLookingToPark = int(round(self.parkingNeedProbability * self.totalVehiclesToLoad))
        logging.info('Total Parking Spots: {}'.format(self.totalVehiclesToPark))
        logging.info('Vehicles Looking To Park: {}'.format(self.totalVehiclesLookingToPark))

    def run(self):
        if self.totalVehiclesLookingToPark < self.totalVehiclesToPark:
            logging.warning('Invalid input: Number of vehicles looking to park is too less.')
            self.traciConnection.close()
            sys.stdout.flush()
            return None
        self.timeStep = 1
        self.wait_for_vehicles_to_load()
        self.initTravelInfo()
        self.selectVehiclesWhichWantToPark()
        self.selectVehiclesWhichWantToRandomRoute()
        self.park_the_initial_set_of_vehicles()
        self.start_simulation()
        self.traciConnection.close()
        return self.compute_results()

    def compute_results(self):
        logging.info('Total Routing Distance: {}, Total Trips: {}'.format(self.totalRoutingDistance, self.totalTrips))
        logging.info('Average Routing Distance: {}'.format(self.totalRoutingDistance / self.totalTrips))
        result = {'averageRoutingDistance': self.totalRoutingDistance / self.totalTrips,
                  'averageRoutingTime': self.totalRoutingTime / self.totalTrips,
                  'averageRoutingDistanceForSmart': self.totalRoutingDistanceForSmart / self.totalTrips,
                  'averageRoutingDistanceForRandom': self.totalRoutingDistanceForRandom / self.totalTrips,
                  'averageRoutingTimeForSmart': self.totalRoutingTimeForSmart / self.totalTrips,
                  'averageRoutingTimeForRandom': self.totalRoutingTimeForRandom / self.totalTrips,
                  'trackedPheromoneLevels': None
                  }
        if self.edgeIDToTrackNeighborsPheromones is not None:
            result['trackedPheromoneLevels'] = self.trackedPhereomoneLevels
        return result

    def start_simulation(self):
        self.maxSteps += self.timeStep
        while self.traciConnection.simulation.getMinExpectedNumber() > 0 and self.timeStep < self.maxSteps:
            self.traciConnection.simulationStep()
            logging.info("Simulation Step: {}".format(self.timeStep))
            if self.edgeIDToTrackNeighborsPheromones is not None:
                self.trackNeighborsPheromones()
            newPheromones = {}  # sum of pheromones added by any vehicles in this time step
            for edgeID in self.G.nodes():
                newPheromones[edgeID] = 0
            for vehID in self.traciConnection.vehicle.getIDList():
                self.updateTravelledDistance(vehID)
                if (self.travelInfo[vehID]['vehicle_state'] == VehicleState.PARKING_NOT_NEEDED or
                        self.travelInfo[vehID]['routing_mode'] == 'random'):
                    self.random_routing_from_edge_neighbors(vehID)
                else:
                    self.pheromone_based_routing_from_edge_neighbors(vehID)
                currentEdgeID = self.traciConnection.vehicle.getRoadID(vehID)
                if currentEdgeID in self.G:
                    self.fsm(vehID, currentEdgeID, newPheromones)
            self.updatePheromoneLevels(newPheromones)
            self.timeStep += 1

    def fsm(self, vehID, currentEdgeID, newPheromones):
        currentState = self.travelInfo[vehID]['vehicle_state']
        logging.debug('{} - Current state: {}'.format(vehID, currentState))
        nextState = currentState
        if currentState == VehicleState.PARKED:
            pass
        elif currentState == VehicleState.GO_TO_HOME:
            nextState = VehicleState.GOING_TO_HOME
        elif currentState == VehicleState.GOING_TO_HOME:
            nextState = VehicleState.SEARCHING_PARKING_AREA
        elif currentState == VehicleState.SCHEDULED_TO_PARK:
            if self.traciConnection.vehicle.isStoppedParking(vehID):
                logging.info("Vehicle {} which was scheduled to park has stopped at {}.".format(vehID, self.travelInfo[vehID]['scheduled_parking_area']['id']))
                nextState = VehicleState.PARKED
                self.markVehicleAsParked(vehID)
                self.update_finish_metrics(vehID)
                self.addPheromonesToTheLastPath(vehID, newPheromones)
                self.unparkARandomVehicle(vehID)
            else:
                self.travelInfo[vehID]['cooldown_counter'] += 1
                # if the vehicle has waited for the max allowed time and has still not been parked
                if self.travelInfo[vehID]['cooldown_counter'] == self.cooldownAfterScheduled:
                    logging.info('Cooldown period of vehicle {} after getting scheduled has ended.'.format(vehID))
                    nextState = VehicleState.SEARCHING_PARKING_AREA
                    self.travelInfo[vehID]['cooldown_counter'] = 0
                    # unreserve the parking area
                    self.travelInfo[vehID]['scheduled_parking_area']['occupancy'] -= 1
        elif currentState == VehicleState.SEARCHING_PARKING_AREA:
            if self.try_to_schedule_a_parking(currentEdgeID, vehID):
                nextState = VehicleState.SCHEDULED_TO_PARK
        self.travelInfo[vehID]['vehicle_state'] = nextState

    def unparkARandomVehicle(self, vehicleToNotRemove):
        # unpark and move the vehicle from PARKED to PARKING_NOT_NEEDED
        logging.info('Unparking a vehicle other than {}.'.format(vehicleToNotRemove))
        modifiedParkedVehiclesSet = self.parkedVehicles.copy()
        modifiedParkedVehiclesSet.remove(vehicleToNotRemove)
        vehIDToRemove = random.choice(list(modifiedParkedVehiclesSet))
        # double check that this vehicle is really parked
        if self.traciConnection.vehicle.isStoppedParking(vehIDToRemove) and self.travelInfo[vehIDToRemove]['vehicle_state'] == VehicleState.PARKED:
            try:
                self.traciConnection.vehicle.setParkingAreaStop(vehIDToRemove, self.travelInfo[vehIDToRemove]['parked_area']['id'], duration=0)
            except traci.exceptions.TraCIException as e:
                logging.warning(e)
            except Exception as e:
                logging.error(e)
            else:
                # bring a vehicle from PARKING_NOT_NEEDED to SEARCHING_PARKING_AREA
                temp = []
                for vehID, data in self.travelInfo.items():
                    if data['vehicle_state'] == VehicleState.PARKING_NOT_NEEDED:
                        temp.append(vehID)
                newVehicleID = random.choice(temp)
                self.travelInfo[newVehicleID]['vehicle_state'] = VehicleState.SEARCHING_PARKING_AREA
                self.travelInfo[newVehicleID]['routing_mode'] = self.travelInfo[vehIDToRemove]['routing_mode']
                self.update_start_metrics(newVehicleID)
                self.travelInfo[vehIDToRemove]['routing_mode'] = 'random'
                self.travelInfo[vehIDToRemove]['vehicle_state'] = VehicleState.PARKING_NOT_NEEDED
                self.parkedVehicles.remove(vehIDToRemove)
                self.travelInfo[vehIDToRemove]['parked_area']['occupancy'] -= 1
                logging.info('Vehicle {} has been unparked from {}.'.format(vehIDToRemove, self.travelInfo[vehIDToRemove]['parked_area']['id']))
                self.travelInfo[vehIDToRemove]['parked_area'] = None

    def update_start_metrics(self, vehID):
        self.travelInfo[vehID]['start_distance'] = self.travelInfo[vehID]['distance_travelled']
        self.travelInfo[vehID]['finish_distance'] = None
        self.travelInfo[vehID]['start_route_id'] = max(self.traciConnection.vehicle.getRouteIndex(vehID), 0)
        self.travelInfo[vehID]['finish_route_id'] = None
        self.travelInfo[vehID]['start_time'] = self.timeStep
        self.travelInfo[vehID]['finish_time'] = None

    def addPheromonesToTheLastPath(self, vehID, newPheromones):
        # get the edges travelled by the vehicle since it last got parked
        vehRoute = self.traciConnection.vehicle.getRoute(vehID)
        pathLength = 0
        for i in range(self.travelInfo[vehID]['start_route_id'], self.travelInfo[vehID]['finish_route_id'] + 1):
            edgeID = vehRoute[i]
            pathLength += self.G.nodes[edgeID]['length']
        if pathLength == 0:
            logging.warning('0 path length. vehID: {}, start_route_id: {}, finish_route_id: {}'.format(vehID, self.travelInfo[vehID]['start_route_id'], self.travelInfo[vehID]['finish_route_id']))
            return
        # add pheromones on the travelled path
        for i in range(self.travelInfo[vehID]['start_route_id'], self.travelInfo[vehID]['finish_route_id'] + 1):
            edgeID = vehRoute[i]
            newPheromones[edgeID] += 1.0 / pathLength

    def update_finish_metrics(self, vehID):
        self.travelInfo[vehID]['finish_distance'] = self.travelInfo[vehID]['distance_travelled']
        self.travelInfo[vehID]['finish_time'] = self.timeStep
        routingDistance = self.travelInfo[vehID]['finish_distance'] - self.travelInfo[vehID]['start_distance']
        routingTime = self.travelInfo[vehID]['finish_time'] - self.travelInfo[vehID]['start_time']
        self.totalRoutingDistance += routingDistance
        self.totalRoutingTime += routingTime
        if self.travelInfo[vehID]['routing_mode'] == 'random':
            self.totalRoutingDistanceForRandom += routingDistance
            self.totalRoutingTimeForRandom += routingTime
        else:
            self.totalRoutingDistanceForSmart += routingDistance
            self.totalRoutingTimeForSmart += routingTime
        self.totalTrips += 1
        # mark the current position on the route
        self.travelInfo[vehID]['finish_route_id'] = self.traciConnection.vehicle.getRouteIndex(vehID)

    def wait_for_vehicles_to_load(self):
        loadedVehicles = 0
        while self.traciConnection.simulation.getMinExpectedNumber() > 0 and loadedVehicles < self.totalVehiclesToLoad:
            self.traciConnection.simulationStep()
            loadedVehicles = self.traciConnection.vehicle.getIDCount()
            logging.info("Simulation Step: {}, loaded vehicles: {}".format(self.timeStep, loadedVehicles))
            for vehID in self.traciConnection.vehicle.getIDList():
                self.random_routing_from_edge_neighbors(vehID)
            self.timeStep += 1
        logging.info('All vehicles have been loaded into the network.')

    def initTravelInfo(self):
        self.travelInfo = {}
        for vehID in self.traciConnection.vehicle.getIDList():
            self.travelInfo[vehID] = {'vehicle_state': VehicleState.PARKING_NOT_NEEDED,
                                      'parked_area': None,
                                      'scheduled_parking_area': None,
                                      'start_distance': 0,
                                      'finish_distance': None,
                                      'start_route_id': 0,
                                      'finish_route_id': None,
                                      'cooldown_counter': 0,
                                      'distance_travelled': 0,
                                      'start_time': 0,
                                      'finish_time': None,
                                      'routing_mode': 'random'}

    def selectVehiclesWhichWantToPark(self):
        loadedVehicles = list(self.traciConnection.vehicle.getIDList())
        random.shuffle(loadedVehicles)
        for i in range(self.totalVehiclesLookingToPark):
            self.travelInfo[loadedVehicles[i]]['vehicle_state'] = VehicleState.SEARCHING_PARKING_AREA

    def selectVehiclesWhichWantToRandomRoute(self):
        numVehiclesToRandomRoute = int(round(self.totalVehiclesLookingToPark * self.percentToRandomRoute / 100))
        logging.info('percentToRandomRoute: {}, numVehiclesToRandomRoute: {}'.format(self.percentToRandomRoute, numVehiclesToRandomRoute))
        loadedVehicles = list(self.traciConnection.vehicle.getIDList())
        random.shuffle(loadedVehicles)
        for i in range(numVehiclesToRandomRoute):
            self.travelInfo[loadedVehicles[i]]['routing_mode'] = 'random'
        for i in range(numVehiclesToRandomRoute, len(loadedVehicles)):
            self.travelInfo[loadedVehicles[i]]['routing_mode'] = 'pheromone'

    def park_the_initial_set_of_vehicles(self):
        while(self.traciConnection.simulation.getMinExpectedNumber() > 0 and
              len(self.parkedVehicles) < self.totalVehiclesToPark):
            self.traciConnection.simulationStep()
            logging.info("Simulation Step: {}, Parked Vehicles: {}".format(self.timeStep, len(self.parkedVehicles)))
            for vehID in self.traciConnection.vehicle.getIDList():
                self.updateTravelledDistance(vehID)
                self.random_routing_from_edge_neighbors(vehID)
                currentEdgeID = self.traciConnection.vehicle.getRoadID(vehID)
                if currentEdgeID in self.G:
                    self.init_period_fsm(vehID, currentEdgeID)
            self.timeStep += 1
        logging.info("Simulation Step: {}, Parked Vehicles: {}".format(self.timeStep, len(self.parkedVehicles)))
        for vehID in self.traciConnection.vehicle.getIDList():
            if self.travelInfo[vehID]['vehicle_state'] == VehicleState.SEARCHING_PARKING_AREA:
                self.travelInfo[vehID]['vehicle_state'] == VehicleState.GO_TO_HOME
        logging.info("Initial set of vehicles have been parked in the network. Remaining vehicles' state changed to {}.".format(VehicleState.GO_TO_HOME.name))

    def updateTravelledDistance(self, vehID):
        # the value given by self.traciConnection.vehicle.getDistance() becomes negative when the vehicle is parked
        # so we should keep track of this value ourselves whenever its positive or to be safe more than the last reading
        d = self.traciConnection.vehicle.getDistance(vehID)
        if d > self.travelInfo[vehID]['distance_travelled']:
            self.travelInfo[vehID]['distance_travelled'] = d

    def random_routing_from_edge_neighbors(self, vehID):
        vehRoute = self.traciConnection.vehicle.getRoute(vehID)
        currentIndexOnRoute = self.traciConnection.vehicle.getRouteIndex(vehID)
        # check if this is the last edge on this route
        if currentIndexOnRoute == -1 or currentIndexOnRoute == len(vehRoute) - 1:
            currentEdgeID = self.traciConnection.vehicle.getRoadID(vehID)
            if currentEdgeID in self.G:
                nextEdgeID = random.choice(list(self.G[currentEdgeID]))
                try:
                    self.traciConnection.vehicle.setRoute(vehID, [currentEdgeID, nextEdgeID])
                except traci.exceptions.TraCIException as e:
                    logging.warning(e)
                except Exception as e:
                    logging.error(e)

    def init_period_fsm(self, vehID, currentEdgeID):
        currentState = self.travelInfo[vehID]['vehicle_state']
        logging.debug('{} - Current state: {}'.format(vehID, currentState))
        nextState = currentState
        if currentState == VehicleState.PARKED:
            pass
        elif currentState == VehicleState.SEARCHING_PARKING_AREA:
            if len(self.parkedVehicles) < self.totalVehiclesToPark:
                if self.try_to_schedule_a_parking(currentEdgeID, vehID):
                    nextState = VehicleState.SCHEDULED_TO_PARK
        elif currentState == VehicleState.SCHEDULED_TO_PARK:
            if self.traciConnection.vehicle.isStoppedParking(vehID):
                logging.info("Vehicle {} which was scheduled to park has stopped at {}.".format(vehID, self.travelInfo[vehID]['scheduled_parking_area']['id']))
                nextState = VehicleState.PARKED
                self.markVehicleAsParked(vehID)
            else:
                self.travelInfo[vehID]['cooldown_counter'] += 1
                # if the vehicle has waited for the max allowed time and has still not been parked,
                # indicating a failure during parking
                if self.travelInfo[vehID]['cooldown_counter'] == self.cooldownAfterScheduled:
                    logging.info('Cooldown period of vehicle {} after getting scheduled has ended.'.format(vehID))
                    nextState = VehicleState.SEARCHING_PARKING_AREA
                    self.travelInfo[vehID]['cooldown_counter'] = 0
                    # unreserve the parking area
                    self.travelInfo[vehID]['scheduled_parking_area']['occupancy'] -= 1
        self.travelInfo[vehID]['vehicle_state'] = nextState

    def try_to_schedule_a_parking(self, currentEdgeID, vehID):
        # try to park the car on some parking area on this edge with free areas
        if len(self.G.nodes[currentEdgeID]['parking_areas']) == 0:
            return False
        for parkingArea in self.G.nodes[currentEdgeID]['parking_areas']:
            logging.debug("Parking Area: {}, capacity: {}, "
                          "occupancy: {}".format(parkingArea['id'], parkingArea['capacity'], parkingArea['occupancy']))
            if parkingArea['occupancy'] < parkingArea['capacity']:
                try:
                    logging.info('Trying to schedule parking for {} on {} at {}'.format(vehID, currentEdgeID, parkingArea['id']))
                    self.traciConnection.vehicle.setParkingAreaStop(vehID, parkingArea['id'], duration=100000)
                except traci.exceptions.TraCIException as e:
                    logging.warning(e)
                except Exception as e:
                    logging.error(e)
                else:
                    self.travelInfo[vehID]['scheduled_parking_area'] = parkingArea
                    parkingArea['occupancy'] += 1
                    logging.info("Vehicle {} has been scheduled to park at {}.".format(vehID, parkingArea['id']))
                    return True
        return False

    def markVehicleAsParked(self, vehID):
        # update the parking details
        self.travelInfo[vehID]['parked_area'] = self.travelInfo[vehID]['scheduled_parking_area']
        self.travelInfo[vehID]['scheduled_parking_area'] = None
        self.parkedVehicles.add(vehID)
        logging.info("Vehicle {} has been marked as parked".format(vehID))

    def updatePheromoneLevels(self, newPheromones):
        for edgeID, pheromoneLevel in newPheromones.items():
            self.pheromoneLevels[edgeID] = self.phereomoneDecayCoefficient * self.pheromoneLevels[edgeID]
            self.pheromoneLevels[edgeID] += self.phereomoneContributionCoefficient * pheromoneLevel

    def trackNeighborsPheromones(self):
        for neighborEdge in self.G[self.edgeIDToTrackNeighborsPheromones]:
            if self.trackedPhereomoneLevels.get(neighborEdge) is None:
                self.trackedPhereomoneLevels[neighborEdge] = []
            self.trackedPhereomoneLevels[neighborEdge].append(self.pheromoneLevels[neighborEdge])

    def pheromone_based_routing_from_edge_neighbors(self, vehID):
        vehRoute = self.traciConnection.vehicle.getRoute(vehID)
        currentIndexOnRoute = self.traciConnection.vehicle.getRouteIndex(vehID)
        # check if this is the last edge on this route or -1
        if currentIndexOnRoute == -1 or currentIndexOnRoute == len(vehRoute) - 1:
            currentEdgeID = self.traciConnection.vehicle.getRoadID(vehID)
            if currentEdgeID in self.G:
                turnProbability = self.calculate_turn_probability_from_edge_neighbors(currentEdgeID)
                neighborEdges = list(self.G[currentEdgeID])
                probabilities = [0] * len(neighborEdges)
                for i in range(len(neighborEdges)):
                    probabilities[i] = turnProbability[neighborEdges[i]]
                nextEdgeID = np.random.choice(neighborEdges, p=probabilities)
                self.traciConnection.vehicle.setRoute(vehID, [currentEdgeID, nextEdgeID])

    def calculate_turn_probability_from_edge_neighbors(self, edgeID):
        turnProbability = {}
        pheromoneSum = 0
        for neighborEdgeID in self.G[edgeID]:
            pheromoneSum += self.pheromoneLevels[neighborEdgeID]
        for neighborEdgeID in self.G[edgeID]:
            turnProbability[neighborEdgeID] = self.pheromoneLevels[neighborEdgeID] / pheromoneSum
        return turnProbability

if __name__ == "__main__":
    gui = True
    if gui:
        sumoBinary = checkBinary('sumo-gui')
    else:
        sumoBinary = checkBinary('sumo')

    client1 = TraciClient(str(1), sumoBinary, parking_need_probability=0.5,
                          phereomone_contribution_coefficient=200, phereomone_decay_coefficient=0.999,
                          cooldown_after_scheduled=30, max_steps=3000, totalVehiclesToLoad=TOTAL_VEHICLES_TO_LOAD,
                          percentToRandomRoute=0)
    client1.run()
    sys.stdout.flush()
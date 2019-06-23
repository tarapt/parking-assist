import networkx as nx
import numpy as np
import random
import xmltodict

NETWORK_XML_FILE = 'aco.net.xml'
ORIGINAL_ADDITIONAL_XML_FILE = 'aco.original_add.xml'
ADDITIONAL_XML_FILE = 'aco.add.xml'


def get_lane_information_from_xml():
    laneDict = {}
    with open(NETWORK_XML_FILE) as fd:
        doc = xmltodict.parse(fd.read())
        edgeList = doc['net']['edge']
        for edge in edgeList:
            # filter out internal roads
            if edge.get('@from') and edge.get('@to'):
                if type(edge['lane']) == list:
                    lanes = [lane['@id'] for lane in edge['lane']]
                    length = edge['lane'][0]['@length']
                else:
                    lanes = [edge['lane']['@id']]
                    length = edge['lane']['@length']
                for lane in lanes:
                    laneDict[lane] = {'edgeID': edge['@id'], 'length': float(length)}
    return laneDict


# note that the nodes in the graph are the lanes of the road network, it also contains internal lanes
def get_graph_from_xml_connections():
    G = nx.DiGraph()
    with open(NETWORK_XML_FILE) as fd:
        doc = xmltodict.parse(fd.read())
        connectionList = doc['net']['connection']
        for connection in connectionList:
            G.add_edge(connection['@from'] + '_' + connection['@fromLane'], connection['@to'] + '_' + connection['@toLane'])
    return G


class NetworkGenerator:
    def __init__(self, totalParkingSpots):
        self.totalParkingSpots = totalParkingSpots
        self.laneDict = get_lane_information_from_xml()
        self.G = get_graph_from_xml_connections()
        self.initialize_nodes()
        self.add_parking_areas_to_lanes()
        self.distribute_parking_spots()
        self.update_additional_xml()

    def distribute_parking_spots(self):
        # select an edge randomly from those edges which have a parking area
        laneList = []
        lengths = []
        for laneID, datadict in self.G.nodes.items():
            if datadict.get('parking_areas') and len(datadict['parking_areas']) > 0:
                lengths.append(datadict['length'])
                laneList.append(laneID)
        lengths = np.array(lengths, dtype=np.float)
        probabilities = lengths / np.sum(lengths)

        for i in range(self.totalParkingSpots):
            randomLane = np.random.choice(laneList, p=probabilities)
            randomParkingArea = random.choice(self.G.nodes[randomLane]['parking_areas'])
            randomParkingArea['@roadsideCapacity'] = str(int(randomParkingArea['@roadsideCapacity']) + 1)

    def update_additional_xml(self):
        updatedAdditionalXml = None
        with open(ORIGINAL_ADDITIONAL_XML_FILE) as fd:
            doc = xmltodict.parse(fd.read())
            updatedParkingAreas = []
            for lane in self.G.nodes():
                if self.G.nodes[lane].get('parking_areas'):
                    updatedParkingAreas.extend(self.G.nodes[lane]['parking_areas'])
            doc['additional']['parkingArea'] = updatedParkingAreas
            updatedAdditionalXml = xmltodict.unparse(doc, pretty=True)
        if updatedAdditionalXml is not None:
            with open(ADDITIONAL_XML_FILE, 'w') as fd:
                fd.write(updatedAdditionalXml)

    # having onRoad=True will block one of the lanes preventing some vehicle to switch lanes
    def add_parking_areas_to_lanes(self, onRoadValue=False):
        with open(ORIGINAL_ADDITIONAL_XML_FILE) as fd:
            doc = xmltodict.parse(fd.read())
            parkingAreaList = doc['additional']['parkingArea']
            for parkingArea in parkingAreaList:
                parkingArea['@roadsideCapacity'] = 0
                parkingArea['@onRoad'] = onRoadValue
                if float(parkingArea['@endPos']) > 200:
                    parkingArea['@length'] = '80'
                else:
                    parkingArea['@length'] = '40'
                parkingArea['@startPos'] = str(round(float(parkingArea['@endPos']) - float(parkingArea['@length']), 2))
                laneID = parkingArea['@lane']
                self.G.nodes[laneID]['parking_areas'].append(parkingArea)
                self.G.nodes[laneID]['length'] = self.laneDict[laneID]['length']

    # initialize the edges, only the non internal nodes have parking
    def initialize_nodes(self):
        for laneID, datadict in self.G.nodes.items():
            datadict['is_internal'] = True if laneID[0] == ':' else False
            if not datadict['is_internal']:
                datadict['parking_areas'] = []
                datadict['parent_edge'] = self.laneDict[laneID]['edgeID']

    def print_graph(self, exclude_internal_nodes=False):
        for laneID, datadict in self.G.nodes.items():
            if exclude_internal_nodes:
                if datadict['is_internal'] is False:
                    print(laneID, datadict, list(self.G[laneID].keys()))
            else:
                print(laneID, datadict, list(self.G[laneID].keys()))


if __name__ == "__main__":
    ng = NetworkGenerator(totalParkingSpots=100)
    ng.print_graph()

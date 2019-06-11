import networkx as nx
import random
import xmltodict

NETWORK_XML_FILE = 'aco.net.xml'
ADDITIONAL_XML_FILE = 'aco.add.xml'
GRAPH_PICKLED_FILE_SAVE_LOCATION = 'graph.gpickle'

random.seed(0)
TOTAL_PARKING_SPOTS = 500
PERCENT_OF_VEHICLES_TO_PARK = 30


def distribute_parking_spots(G):
    # select an edge randomly from those edges which have a parking area
    laneList = [n for n in G.nodes() if G.nodes[n].get('parking_areas') and len(G.nodes[n]['parking_areas']) > 0]
    for i in range(TOTAL_PARKING_SPOTS):
        randomLane = random.choice(laneList)
        randomParkingArea = random.choice(G.nodes[randomLane]['parking_areas'])
        randomParkingArea['@roadsideCapacity'] = str(int(randomParkingArea['@roadsideCapacity']) + 1)


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
                    laneDict[lane] = {'edgeID': edge['@id'], 'length': length}
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


def update_additional_xml(G):
    updatedAdditionalXml = None
    with open(ADDITIONAL_XML_FILE) as fd:
        doc = xmltodict.parse(fd.read())
        updatedParkingAreas = []
        for lane in G.nodes():
            if G.nodes[lane].get('parking_areas'):
                updatedParkingAreas.extend(G.nodes[lane]['parking_areas'])
        doc['additional']['parkingArea'] = updatedParkingAreas
        updatedAdditionalXml = xmltodict.unparse(doc, pretty=True)
    if updatedAdditionalXml is not None:
        with open(ADDITIONAL_XML_FILE, 'w') as fd:
            fd.write(updatedAdditionalXml)


# having onRoad=True will block one of the lanes preventing some vehicle to switch lanes
def add_parking_areas_to_lanes(G, laneDict, onRoadValue=False):
    with open(ADDITIONAL_XML_FILE) as fd:
        doc = xmltodict.parse(fd.read())
        parkingAreaList = doc['additional']['parkingArea']
        for parkingArea in parkingAreaList:
            parkingArea['@roadsideCapacity'] = str(int(onRoadValue))
            parkingArea['@onRoad'] = onRoadValue
            laneID = parkingArea['@lane']
            G.nodes[laneID]['parking_areas'].append(parkingArea)
            G.nodes[laneID]['length'] = laneDict[laneID]['length']


'''
# not needed as alternative logic exists for keeping number of parked vehicles constant
def distribute_parked_vehicles(G, percentOfVehiclesToPark):
    totalVehiclesToPark = round(percentOfVehiclesToPark / 100 * TOTAL_PARKING_SPOTS)
    for i in range(totalVehiclesToPark):
        parkingSpots = []
        for edge, datadict in G.edges.items():
            availableParkingSpots = datadict['parking_capacity'] - datadict['parked_vehicles_count']
            if availableParkingSpots > 0:
                parkingSpots.append(edge)
        randomEdge = random.choice(parkingSpots)
        G.edges[randomEdge]['parked_vehicles_count'] += 1
'''

# initialize the edges, only the non internal nodes have parking


def initialize_nodes(laneDict):
    for laneID, datadict in G.nodes.items():
        datadict['is_internal'] = True if laneID[0] == ':' else False
        if not datadict['is_internal']:
            datadict['parking_areas'] = []
            datadict['parent_edge'] = laneDict[laneID]['edgeID']


def print_graph(G, exclude_internal_nodes=False):
    for laneID, datadict in G.nodes.items():
        if exclude_internal_nodes:
            if datadict['is_internal'] is False:
                print(laneID, datadict, list(G[laneID].keys()))
        else:
            print(laneID, datadict, list(G[laneID].keys()))


if __name__ == "__main__":
    laneDict = get_lane_information_from_xml()
    G = get_graph_from_xml_connections()
    initialize_nodes(laneDict)
    add_parking_areas_to_lanes(G, laneDict)
    distribute_parking_spots(G)
    update_additional_xml(G)
    # print_graph(G)
    nx.write_gpickle(G, GRAPH_PICKLED_FILE_SAVE_LOCATION)

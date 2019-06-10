import networkx as nx
import random
import xmltodict

NETWORK_XML_FILE = '../sumo/aco.net.xml'
ADDITIONAL_XML_FILE = '../sumo/aco.add.xml'
GRAPH_PICKLED_FILE_SAVE_LOCATION = 'graph.gpickle'

random.seed(0)
TOTAL_PARKING_SPOTS = 500
PERCENT_OF_VEHICLES_TO_PARK = 30


def distribute_parking_spots(G):
    # select an edge randomly from those edges which have a parking area
    nodeList = [n for n in G.nodes() if len(G.nodes[n]['parking_areas']) > 0]
    for i in range(TOTAL_PARKING_SPOTS):
        randomNode = random.choice(nodeList)
        randomParkingArea = random.choice(G.nodes[randomNode]['parking_areas'])
        randomParkingArea['@roadsideCapacity'] = str(int(randomParkingArea['@roadsideCapacity']) + 1)


def set_on_road_parking(value):
    updatedAdditionalXml = None
    with open(ADDITIONAL_XML_FILE) as fd:
        doc = xmltodict.parse(fd.read())
        parkingAreaList = doc['additional']['parkingArea']
        for parkingArea in parkingAreaList:
            parkingArea['@onRoad'] = str(value)
        updatedAdditionalXml = xmltodict.unparse(doc, pretty=True)

    with open(ADDITIONAL_XML_FILE, 'w') as fd:
        fd.write(updatedAdditionalXml)


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


def get_graph_from_xml_connections():
    G = nx.DiGraph()
    with open(NETWORK_XML_FILE) as fd:
        doc = xmltodict.parse(fd.read())
        connectionList = doc['net']['connection']
        for connection in connectionList:
            G.add_edge(connection['@from'], connection['@to'])
    return G

# def distribute_parked_vehicles(G, percentOfVehiclesToPark):
#     totalVehiclesToPark = round(percentOfVehiclesToPark / 100 * TOTAL_PARKING_SPOTS)
#     for i in range(totalVehiclesToPark):
#         parkingSpots = []
#         for edge, datadict in G.edges.items():
#             availableParkingSpots = datadict['parking_capacity'] - datadict['parked_vehicles_count']
#             if availableParkingSpots > 0:
#                 parkingSpots.append(edge)

#         randomEdge = random.choice(parkingSpots)
#         G.edges[randomEdge]['parked_vehicles_count'] += 1


if __name__ == "__main__":
    laneDict = get_lane_information_from_xml()

    # note that the nodes in the graph are the roads of the road network, it also contains internal roads
    G = get_graph_from_xml_connections()

    # initialize the edges, only the non internal nodes have parking
    for node, datadict in G.nodes.items():
        datadict['is_internal'] = True if node[0] == ':' else False
        datadict['parked_vehicles_count'] = 0
        datadict['moving_vehicles_count'] = 0
        datadict['parking_areas'] = []
        datadict['parking_capacity'] = 0
    #     edge[2]['pheromone_level'] = 1
    #     edge[2]['selection_probability'] = 0

    updatedAdditionalXml = None
    with open(ADDITIONAL_XML_FILE) as fd:
        doc = xmltodict.parse(fd.read())
        parkingAreaList = doc['additional']['parkingArea']
        for parkingArea in parkingAreaList:
            parkingArea['@roadsideCapacity'] = '0'
            edgeID = laneDict[parkingArea['@lane']]['edgeID']
            length = laneDict[parkingArea['@lane']]['length']
            G.nodes[edgeID]['parking_areas'].append(parkingArea)
            G.nodes[edgeID]['length'] = length

    distribute_parking_spots(G)
    updatedParkingAreas = []
    for node in G.nodes():
        updatedParkingAreas.extend(G.nodes[node]['parking_areas'])
    doc['additional']['parkingArea'] = updatedParkingAreas
    updatedAdditionalXml = xmltodict.unparse(doc, pretty=True)

    with open(ADDITIONAL_XML_FILE, 'w') as fd:
        fd.write(updatedAdditionalXml)

    for n, datadict in G.nodes.items():
        for parkingArea in datadict['parking_areas']:
            datadict['parking_capacity'] += int(parkingArea['@roadsideCapacity'])
        # if not datadict['is_internal']:
        #     print(n, datadict)

    print(random.choice(list(G['gneE0'].keys())))

    # not needed as alternative logic exists for keeping number of parked vehicles constant
    # distribute_parked_vehicles(G, PERCENT_OF_VEHICLES_TO_PARK)

    nx.write_gpickle(G, GRAPH_PICKLED_FILE_SAVE_LOCATION)
    # set_on_road_parking(1)

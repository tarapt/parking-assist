import networkx as nx
import random
import xmltodict

NETWORK_XML_FILE = '../sumo/aco.net.xml'
ADDITIONAL_XML_FILE = '../sumo/aco.add.xml'
GRAPH_PICKLED_FILE_SAVE_LOCATION = 'graph.gpickle'

random.seed(0)
TOTAL_PARKING_SPOTS = 100
PERCENT_OF_VEHICLES_TO_PARK = 30


def distribute_parking_spots(G):
    # select an edge randomly from those edges which have a parking area
    edgeList = [e for e in G.edges() if len(G.edges[e]['parking_areas']) > 0]
    for i in range(TOTAL_PARKING_SPOTS):
        randomEdge = random.choice(edgeList)
        randomParkingArea = random.choice(G.edges[randomEdge]['parking_areas'])
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


def convert_xml_edges_to_networkx():
    G = nx.DiGraph()
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
                    laneDict[lane] = (edge['@from'], edge['@to'])
                G.add_edge(edge['@from'], edge['@to'], id=edge['@id'], length=length, laneIDs=lanes, parking_areas=[])
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


laneDict = {}
G = convert_xml_edges_to_networkx()

# initialize the edges
for edge in G.edges(data=True):
    edge[2]['parking_capacity'] = 0
    edge[2]['parked_vehicles_count'] = 0
    edge[2]['moving_vehicles_count'] = 0
#     edge[2]['pheromone_level'] = 1
#     edge[2]['selection_probability'] = 0

updatedAdditionalXml = None
with open(ADDITIONAL_XML_FILE) as fd:
    doc = xmltodict.parse(fd.read())
    parkingAreaList = doc['additional']['parkingArea']
    for parkingArea in parkingAreaList:
        parkingArea['@roadsideCapacity'] = '0'
        edge = laneDict[parkingArea['@lane']]
        G.edges[edge]['parking_areas'].append(parkingArea)

    distribute_parking_spots(G)
    updatedParkingAreas = []
    for edge in G.edges():
        updatedParkingAreas.extend(G.edges[edge]['parking_areas'])
    doc['additional']['parkingArea'] = updatedParkingAreas
    updatedAdditionalXml = xmltodict.unparse(doc, pretty=True)


for e, datadict in G.edges.items():
    for parkingArea in datadict['parking_areas']:
        datadict['parking_capacity'] += int(parkingArea['@roadsideCapacity'])

# not needed as alternative logic exists for keeping number of parked vehicles constant
# distribute_parked_vehicles(G, PERCENT_OF_VEHICLES_TO_PARK)

with open(ADDITIONAL_XML_FILE, 'w') as fd:
    fd.write(updatedAdditionalXml)

nx.write_gpickle(G, GRAPH_PICKLED_FILE_SAVE_LOCATION)

set_on_road_parking(1)

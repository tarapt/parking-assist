import networkx as nx
import random
import xmltodict

EDGES_XML_FILE = '../sumo/aco.edg.xml'
GRAPH_PICKLED_FILE_SAVE_LOCATION = 'graph.gpickle'

random.seed(0)
TOTAL_PARKING_SPOTS = 20


def distribute_parking_spots(G):
    edgeList = [e for e in G.edges()]
    for i in range(TOTAL_PARKING_SPOTS):
        randomEdge = random.choice(edgeList)
        G.edges[randomEdge]['parking_capacity'] += 1


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


with open(EDGES_XML_FILE) as fd:
    G = nx.DiGraph()
    doc = xmltodict.parse(fd.read())
    for x in doc['edges']['edge']:
        G.add_edge(x['@from'], x['@to'], id=x['@id'])

    # initialize the edges
    for edge in G.edges(data=True):
        edge[2]['parking_capacity'] = 0
        edge[2]['parked_vehicles_count'] = 0
        edge[2]['moving_vehicles_count'] = 0
        edge[2]['pheromone_level'] = 1
        edge[2]['selection_probability'] = 0

    distribute_parking_spots(G)
    distribute_parked_vehicles(G, 80)

    for e, datadict in G.edges.items():
        print(e, datadict)

    nx.write_gpickle(G, GRAPH_PICKLED_FILE_SAVE_LOCATION)

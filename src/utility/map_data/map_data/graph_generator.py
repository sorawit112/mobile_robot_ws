import networkx as nx
import os, glob, pathlib
import json
import math

class GraphLoader:
    def __init__(self, map_name):
        self.map_name = map_name
        self.graph = nx.Graph(name=map_name)

        self.nodes, self.edges, self.stations = self.load_metadata(map_name)

        self.graph.add_nodes_from(self.nodes.keys())
        for n,p in self.nodes.items():
            self.graph.nodes[n]['pos'] = p
        for e,d in self.edges.items():
            self.graph.add_edge(d[0],d[1],weight=euc2d(self.nodes[d[0]],self.nodes[d[1]]))

    def load_metadata(self, map_name):
        path = pathlib.Path(__file__).parent.resolve()
        path = os.path.dirname(path)
        file = glob.glob(path + '/map_data/' + str(map_name) + '/metadata.json')
        with open(file[0],"r") as json_data:
            try:
                data = json.load(json_data)
            except json.JSONDecodeError as exec:
                print(exec)
        dict_nodes = {}
        dict_edges = {}
        dict_stations = {}

        for n,p in data['nodes'].items():
            dict_nodes[int(n)] = tuple(p)
        for e,d in data['edges'].items():
            dict_edges[int(e)] = tuple(d)
        for s,n in data['stations'].items():
            dict_stations[str(s)] = int(n)

        return dict_nodes, dict_edges, dict_stations

def euc2d(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)    


if __name__=="__main__":
    graph_loader = GraphLoader('map_01')
    print(graph_loader.nodes)
    print(graph_loader.edges)
    print(graph_loader.stations)
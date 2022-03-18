from task_manager.worker_manager import Worker
import networkx as nx
import os, glob, pathlib
import json
import math

class GraphLoader:
    def __init__(self, map_name):
        self.map_name = map_name
        self._graph = nx.Graph(name=map_name)
        self._nodes, self._edges, self._edges_task, self._stations = self.load_metadata(map_name)

        self._graph.add_nodes_from(self._nodes.keys())
        for n,p in self._nodes.items():
            self._graph.nodes[n]['pos'] = p
        for e,d in self._edges.items():
            self._graph.add_edge(d[0],d[1],weight=euc2d(self._nodes[d[0]],self._nodes[d[1]]))

    def load_metadata(self, map_name):
        # path = pathlib.Path(__file__).parent.resolve()
        # path = os.path.dirname(path)
        file = glob.glob('/home/trainee/ros2_ws/mobile_robot_ws/src/utility/map_data/map_data/map_01/metadata.json')
        with open(file[0],"r") as json_data:
            try:
                data = json.load(json_data)
            except json.JSONDecodeError as exec:
                print(exec)
        dict_nodes = {}
        dict_edges = {}
        dict_edges_task = {}
        dict_stations = {}
        
        for n,p in data['nodes'].items():
            dict_nodes[int(n)] = tuple(p)
        for e,d in data['edges'].items():
            dict_edges[int(e)] = tuple(d[0])
            dict_edges_task[int(e)] = Worker._dict.value[str(d[1])]
        for s,n in data['stations'].items():
            dict_stations[str(s)] = int(n)

        return dict_nodes, dict_edges, dict_edges_task, dict_stations

def euc2d(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)    


if __name__=="__main__":
    graph_loader = GraphLoader('map_01')
    print(graph_loader._nodes)
    print(graph_loader._edges)
    print(graph_loader._stations)
    print(graph_loader._graph.edges())
    print(graph_loader._edges_task)
from task_manager.worker_manager import Worker
from queue import Queue
import networkx as nx

from custom_msgs.msg import MapMetadata
from custom_msgs.srv import GetMapMetadata

class Task(object):
    def __init__(self, start, goal, worker):
        self.node_start = start
        self.node_goal = goal
        self.worker = worker

class GraphPlanner(object):
    def __init__(self, node):
        self.module_name = 'Graph Planner'
        self.node = node

        self.graph = nx.Graph(name="metadata") #graph._graph
        self.nodes = {} #graph._nodes
        self.edges = {} #graph._edges
        self.edges_task = {} #graph._edges_task
        self.stations = {} #graph._stations

        self.tasks = Queue(0) #infinit size

    def set_map_metadata(self, metadata):
        node_list = metadata.nodes
        edge_list = metadata.edges
        station_list = metadata.stations

        for node in node_list:
            self.nodes[node.id] = (node.point.x, node.point.y)
            
        self.graph.add_nodes_from(self.nodes.keys())
        for n,p in self.nodes.items():
            self.graph.nodes[n]['pos'] = p

        for edge in edge_list:
            self.edges[edge.id] = [edge.src, edge.dst, edge.weight]
            self.edges_task[edge.id] = edge.worker
            self.graph.add_edge(edge.src, edge.dst, weight = edge.weight)

        for station in station_list:
            self.stations[station.name] = station.node_id

        self.do_logging("set_map_metada finished !!")

    def plan(self, node_current, station_start, station_goal):
        node_start = self.stations[str(station_start).upper()]
        node_goal = self.stations[str(station_goal).upper()]

        self.do_logging("**Plan** from station {}(node {}) --> station {}(node {})".format(station_start,
                                                                                             node_start,
                                                                                             station_goal,
                                                                                             node_goal))
        # path from current --> start
        pathFound = True
        try:
            via_points = nx.astar_path(self.graph, node_current, node_start) 
        except nx.NetworkXError as exec:
            pathFound = False
            self.do_logging(exec)

        if not pathFound:
            self.do_logging("plan !!FAILED")
            return False

        # path from start --> goal    
        try:
            via_points = via_points + nx.astar_path(self.graph, node_start, node_goal)[1:] 
        except nx.NetworkXError as exec:
            pathFound = False
            self.do_logging(exec)

        if not pathFound:
            self.do_logging("plan !!FAILED")
            return False

        for i in range(len(via_points)-1):
            s = via_points[i]
            g = via_points[i+1]

            edge = self.edge_from_nodes(s,g)
            worker = self.edges_task[edge]
            task = Task(s,g,worker)
            self.tasks.put(task)           
        
        self.do_logging("plan !!SUCCEED")
        return True

    def edge_from_nodes(self, s, g):
        node_in_edge = tuple(sorted([s,g]))
        return list(self.graph.edges).index(node_in_edge)

    def get(self):
        self.do_logging('peek first one from tasks queue')
        return self.tasks.get_nowait()
        
    def do_logging(self, msg):
        self.node.get_logger().info("[{0}] {1}".format(self.module_name, msg))

if __name__ == "__main__":
    plan = GraphPlanner('temp_node', "map_01")
    plan.plan(9,'A','e')
    while not plan.tasks.empty():
        task = plan.tasks.get()
        print(task.node_start, task.node_goal, task.worker)
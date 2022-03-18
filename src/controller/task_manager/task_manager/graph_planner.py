from task_manager.worker_manager import Worker
from queue import Queue
import networkx as nx

from map_data.graph_generator import GraphLoader

class Task(object):
    def __init__(self, start, goal, worker):
        self.node_start = start
        self.node_goal = goal
        self.worker = worker

class GraphPlanner(object):
    def __init__(self, node, map_name):
        self.module_name = 'Graph Planner'
        self.node = node
        graph = GraphLoader(map_name)
        self.graph = graph._graph
        self.nodes = graph._nodes
        self.edges = graph._edges
        self.edges_task = graph._edges_task
        self.stations = graph._stations

        self.tasks = Queue(0) #infinit size

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
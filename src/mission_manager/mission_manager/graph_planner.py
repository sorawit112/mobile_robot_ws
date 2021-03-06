from queue import Queue
import networkx as nx
import numpy as np

class Task(object):
    def __init__(self, src_node, dst_node, src_pose, dst_pose, unit):
        self.src_node = src_node
        self.dst_node = dst_node
        self.src_pose = src_pose
        self.dst_pose = dst_pose
        self.via_points = []
        self.unit = unit

class GraphPlanner(object):
    def __init__(self, node):
        self.module_name = 'Graph Planner'
        self.node = node

        self.graph = nx.Graph(name="metadata") #graph._graph
        self.nodes = {} 
        self.edges = {} 
        self.edges_task = {} 
        self.stations = {} 

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
            self.edges_task[edge.id] = edge.unit
            self.graph.add_edge(edge.src, edge.dst, weight = edge.weight)

        for station in station_list:
            self.stations[station.name] = station.node_id

        self.info("set_map_metada finished !!")

    def plan(self, current_node, node_list):
        if current_node != node_list[0]:
            node_list[0] = current_node
        
        via_points = []
        first = True
        for i in range(len(node_list)-1):
            node_start = node_list[i]
            node_goal = node_list[i+1]
            self.info("**Plan** from (node {}) --> (node {})".format(node_start,
                                                                     node_goal))
            try:
                via = nx.astar_path(self.graph, node_start, node_goal) 
                # print(via)
                if first:
                    via_points = via_points + via
                    first = False
                else:
                    via_points = via_points + via[1:]
            except nx.NetworkXError as err:
                self.error(err)
                self.error("plan !!FAILED")
                return False
            
        print(via_points)
        for i in range(len(via_points)-1):
            src_node = via_points[i]
            dst_node = via_points[i+1]
            src_pose = self.nodes[src_node]
            dst_pose = self.nodes[dst_node]

            edge = self.edge_from_nodes(src_node, dst_node)

            unit = self.edges_task[edge]
            task = Task(src_node, dst_node, src_pose, dst_pose, unit)
            self.tasks.put(task)      
        
        self.info("plan !!SUCCEED")
        return True

    def nearest_node_from_pose(self, pose):
        current = np.array([pose.position.x, pose.position.y])
        min_dist = 999

        for n,p in self.nodes.items():
            dist = np.linalg.norm(current - np.array(p))
            if dist < min_dist:
                min_node_dist = n
                min_dist = dist

        self.info("Nearest Node[{1}] from pose : {0}".format((pose.position.x, pose.position.y), min_node_dist))
        return min_node_dist

    def edge_from_nodes(self, s, g):
        node_in_edge = tuple(sorted([s,g]))
        return list(self.graph.edges).index(node_in_edge)

    def get(self):
        """Pop first task from queue"""
        self.info('Pop first task from queue')
        return self.tasks.get_nowait()

    def peek(self):
        """Peek first task from queue"""
        
        if not self.tasks.empty():
            self.info('Peek first task from queue')
            return self.tasks[0], True
        else:
            self.warn('Queue is empty cant peek!!')
            return None, False

    def show_task(self, task, unit_name):
        self.info(f'src_node : {task.src_node}')
        self.info(f'dst_node : {task.dst_node}')
        self.info(f'unit : {unit_name}')

    def clear(self):
        """Clear Tasks Queue"""
        self.info('Clear Current Task Queue')
        self.tasks = Queue(0)

    ########################################################################################
    #############           Logging
    ########################################################################################
    def info(self, msg):
        self.node.get_logger().info(msg)
        return

    def warn(self, msg):
        self.node.get_logger().warn(msg)
        return

    def error(self, msg):
        self.node.get_logger().error(msg)
        return

    def debug(self, msg):
        self.node.get_logger().debug(msg)
        return

if __name__ == "__main__":
    plan = GraphPlanner('temp_node', "map_01")
    plan.plan(9,'A','e')
    while not plan.tasks.empty():
        task = plan.tasks.get()
        print(task.src_node, task.dst_node, task.unit)
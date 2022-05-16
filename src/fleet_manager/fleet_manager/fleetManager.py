import rclpy
from rclpy.node import Node

from mission_manager.topics import Topics
from custom_msgs.srv import GetMapMetadata
from custom_msgs.msg import MapMetadata

from queue import Queue
import networkx as nx
from networkx import DiGraph, from_numpy_matrix, relabel_nodes, set_node_attributes
from vrpy import VehicleRoutingProblem

import matplotlib.pyplot as plt

import numpy as np

class TraverseGraph(object):
    def __init__(self, ros_node):
        self.a_graph = nx.Graph(name="traverse") 
        self.ros_node = ros_node

        self.depot_node = None
        self.nodes = {} 
        self.edges = {} 
        self.edges_task = {} 
        self.stations = {} 

    def set_map_metadata(self, metadata):
        node_list = metadata.nodes
        edge_list = metadata.edges
        station_list = metadata.stations
        self.depot_node = metadata.depot_node

        for node in node_list:
            self.nodes[node.id] = (node.point.x, node.point.y)
            
        self.a_graph.add_nodes_from(self.nodes.keys())
        for n,p in self.nodes.items():
            self.a_graph.nodes[n]['pos'] = p

        for edge in edge_list:
            self.edges[edge.id] = [edge.src, edge.dst, edge.weight]
            self.edges_task[edge.id] = edge.unit
            self.a_graph.add_edge(edge.src, edge.dst, weight = edge.weight)

        for station in station_list:
            self.stations[station.name] = station.node_id

        self.ros_node.info("set_map_metada finished !!")

    def create_distance_matrix(self):
        # nodes = list(self.nodes.keys())
        nodes = list(self.stations.values())
        adjacency_dict = {}
        n_mat = len(nodes)+1
        # adj_matrix = np.zeros((len(nodes),len(nodes)), dtype=np.int32)
        # n_sink_vec = np.zeros((len(nodes),1), dtype=np.int32)
        adj_matrix = np.zeros((n_mat, n_mat), dtype=np.int32)
        n_sink_vec = np.zeros((n_mat,1), dtype=np.int32)

        #adjacency dict
        # for i,n in enumerate(nodes):
        #     n1 = n
        #     adjacency_dict[n1] = {}
        #     for j in range(i+1, len(nodes)):
        #         n2 = nodes[j]
        #         length = nx.astar_path_length(self.a_graph, n1, n2)
        #         adjacency_dict[n1][n2] = int(round(length, 1)*10)

        for i,n in enumerate(nodes):
            n1 = n
            adjacency_dict[i] = {}
            for j in range(i+1, len(nodes)):
                n2 = nodes[j]
                length = nx.astar_path_length(self.a_graph, n1, n2)
                adjacency_dict[i][j] = int(round(length, 2)*100)

            length = nx.astar_path_length(self.a_graph, n1, self.depot_node)
            adjacency_dict[i][j+1] = int(round(length, 2)*100)

        adjacency_dict[i+1] = {}

        print(adjacency_dict)

        #construct adjacency matrix
        
        for n1, inner_dict in adjacency_dict.items():
            for n2 in inner_dict.keys():
                if n2 == list(adjacency_dict.keys())[-1]:
                    adj_matrix[n1][n2] = 0 #node_to_source = 0
                    adj_matrix[n2][n1] = inner_dict[n2] #source_to_node

                    #backup node_to_sink then stack after
                    n_sink_vec[n1] = inner_dict[n2]
                else:
                    adj_matrix[n1][n2] = inner_dict[n2]
                    adj_matrix[n2][n1] = inner_dict[n2]

        # stack node to sink  
        adj_matrix = np.hstack((adj_matrix, n_sink_vec)) 

        # stack source to sink
        adj_matrix = np.vstack((adj_matrix, np.zeros((1,n_mat+1), dtype=np.int32)))        
        
        #visualize matrix
        for _, inner in enumerate(adj_matrix):
            print('|', end = " ")
            for val in inner:
                v = str(val)
                for i in range(4-len(v), 0, -1):
                    print(" ", end="")
                print(v, end=" ")
            print('|')
            
        # new_adj_matrix, bot_mat = np.vsplit(adj_matrix, [45])
        # new_adj_matrix, right_mat = np.hsplit(new_adj_matrix, [45])

        # right_mat = np.transpose(right_mat)
        # right_mat_0 = right_mat[0].reshape(new_adj_matrix.shape[0], 1)
        # right_mat_1 = right_mat[1].reshape(new_adj_matrix.shape[0], 1)

        # new_adj_matrix = np.hstack((right_mat_0, new_adj_matrix, right_mat_1))
        # bot_mat_0 = np.hstack((np.array([0]),bot_mat[0][0:45],np.array([0])))
        # new_adj_matrix = np.vstack((bot_mat_0, new_adj_matrix, bot_mat[1]))
        
        # #visualize matrix
        # for _, inner in enumerate(new_adj_matrix):
        #     print('|', end = " ")
        #     for val in inner:
        #         v = str(val)
        #         for i in range(3-len(v), 0, -1):
        #             print(" ", end="")
        #         print(v, end=" ")
        #     print('|')

        return adj_matrix #new_adj_matrix


class FleetManager(Node):
    def __init__(self):
        super().__init__(node_name="FleetManager")
        self.module_name = 'FleetManager'
        self.topic = Topics()

        self.traverse_graph = TraverseGraph(self)
        self.adj_matrix = None
        self.vrp_graph = None
        self.depot_node = None
        self.metadata = MapMetadata()

        self.n_robots = 2

    def pickup_delivery_plan(self, pickups_list, deliveries_list, demands_list):
        if len(pickups_list) != len(deliveries_list) and len(pickups_list) != len(demands_list):
            self.error('number of pickup-delivery input list not equal')
            return False

        #get list of all stations input
        station_list = [n for n in pickups_list]
        for n in deliveries_list:
            if n in station_list:
                self.error(f'station {n} is visit more than once')
                return False
            station_list.append(n)

        
            
        

        for i, from_node in enumerate(pickups_list):
            from_node = self.traverse_graph.stations[from_node] #+ 1
            to_node = self.traverse_graph.stations[deliveries_list[i]] #+ 1
            demand = demands_list[i]
            self.vrp_graph.nodes[from_node]["request"] = to_node
            # Pickups are accounted for positively
            self.vrp_graph.nodes[from_node]["demand"] = demand
            # Deliveries are accounted for negatively
            self.vrp_graph.nodes[to_node]["demand"] = -demand

        print(f"demand : {nx.get_node_attributes(self.vrp_graph, 'demand')}")
        print(f"request : {nx.get_node_attributes(self.vrp_graph, 'request')}")

        # nx.draw_shell(self.vrp_graph, with_labels=True)
        # plt.show()

        prob = VehicleRoutingProblem(self.vrp_graph, 
                                     load_capacity=4, 
                                    #  num_vehicles=self.n_robots, 
                                     pickup_delivery=True,
                                     num_stops=4)
        try:
            prob.solve(cspy=False, exact=True, pricing_strategy="BestPaths")
            print(prob.best_routes)
            print(prob.node_load)
            self.info("plan !!success")

            return True
        except Exception as e:
            print(e)
            print(prob.G.nodes())
            self.info("plan !!Failed")

            return False
        
    def request_map_metadata(self, time_out=5):
        self.info('wait for get map metdata service')
        get_map_client = self.create_client(GetMapMetadata, self.topic.get_map_metada)
        srv_ready = get_map_client.wait_for_service(timeout_sec=time_out)
        if not srv_ready:
            self.info('get map metdata service not ready')
            return False

        self.info('receive response from server')
        future = get_map_client.call_async(GetMapMetadata.Request())
        rclpy.spin_until_future_complete(self, future)

        self.metadata = future.result().metadata

        return True

    def construct_vrp_graph(self, station_list):
        self.traverse_graph.set_map_metadata(self.metadata)

        adj_matrix = self.traverse_graph.create_distance_matrix() #get fully connected all station

        self.adj_matrix = np.array(adj_matrix, dtype=[("cost", int)])
        self.vrp_graph = from_numpy_matrix(self.adj_matrix, create_using=nx.DiGraph())
        
        # for u,v,w in self.vrp_graph.edges(data="cost"):
        #     print(f"from:{u} to:{v} weight={w}")

        # The depot is relabeled as Source and Sink
        self.vrp_graph = relabel_nodes(self.vrp_graph, 
                                      {adj_matrix.shape[0]-2: "Source", #0: "Source", 
                                       adj_matrix.shape[0]-1: "Sink"})

        # for n in range(len(nx.nodes(self.vrp_graph))-2):
        #     self.vrp_graph.nodes[n]["demand"] = 1
 
        print("")
        print(f"nodes : {nx.nodes(self.vrp_graph)}")

        return True

    ########################################################################################
    #############           Logging
    ########################################################################################
    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

def main(args=None):
    rclpy.init(args=args)
    pickups_list =    ['A', 'C', 'E', 'G', 'I', 'K', 'M'] #, 'F', 'G', 'H', 'D']
    deliveries_list = ['B', 'D', 'F', 'H', 'J', 'L', 'N'] #, 'K', 'J', 'B', 'N']
    demand_list =     [ 1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ] #,  1 ,  1 ,  1 ,  1 ]

    fleet_manager = FleetManager()

    fleet_manager.request_map_metadata()
    fleet_manager.pickup_delivery_plan(pickups_list,
                                       deliveries_list,
                                       demand_list)
    try:
        rclpy.spin(fleet_manager)
    except:
        fleet_manager.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
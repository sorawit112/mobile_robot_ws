import rclpy
from rclpy.node import Node

from mission_manager.topics import Topics
from custom_msgs.srv import GetMapMetadata, UserMission
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

        self.adjacency_dict = None
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

    def create_adjacency_dict(self):
        nodes = list(self.stations.values())
        adjacency_dict = {}
        
        for i,n in enumerate(nodes):
            n1 = n
            adjacency_dict[i] = {}
            for j in range(i+1, len(nodes)):
                n2 = nodes[j]
                length = nx.astar_path_length(self.a_graph, n1, n2)
                adjacency_dict[i][j] = int(round(length, 2)*100)

            length = nx.astar_path_length(self.a_graph, n1, self.depot_node)
            adjacency_dict[i][j+1] = int(round(length, 2)*100)

        adjacency_dict[len(nodes)] = {}

        self.adjacency_dict = adjacency_dict

    def create_adjacency_matrix(self, station_list):
        n_mat = len(station_list)+1
        #construct adjacency matrix
        adj_matrix = np.zeros((n_mat, n_mat), dtype=np.int32)
        n_sink_vec = np.zeros((n_mat,1), dtype=np.int32)

        #selected stations from fully adjacency dict
        adjacency_dict = {}
        remap_station_dict = {} #remap node by sorted station
        filtered_station_dict = {}

        for i, name in enumerate(station_list):
            node = self.stations[name]
            remap_station_dict[name] = i
            adjacency_dict[i] = self.adjacency_dict[node]
            filtered_station_dict[node] = i

        adjacency_dict[n_mat-1] = {} #add depot_node inner dict
        filtered_station_dict[len(self.stations)] = n_mat-1

        print("fully connected adjacency_dict")
        print(self.adjacency_dict)
        print("=========================================================================")
        print("slected adjacency_dict")
        print(adjacency_dict)
        print("=========================================================================")

        for n1, inner_dict in adjacency_dict.items():
            # print(n1, inner_dict.keys())
            for n2 in inner_dict.keys():
                if n2 not in filtered_station_dict.keys():
                    continue
                fil_n = filtered_station_dict[n2]
                # print(n1,n2,fil_n)
                if n2 == list(self.adjacency_dict.keys())[-1]:
                    adj_matrix[n1][fil_n] = 0 #node_to_source = 0
                    adj_matrix[fil_n][n1] = inner_dict[n2] #source_to_node

                    #backup node_to_sink then stack after
                    n_sink_vec[n1] = inner_dict[n2]
                else:
                    adj_matrix[n1][fil_n] = inner_dict[n2]
                    adj_matrix[fil_n][n1] = inner_dict[n2]

        # stack node to sink  
        adj_matrix = np.hstack((adj_matrix, n_sink_vec)) 

        # stack source to sink
        adj_matrix = np.vstack((adj_matrix, np.zeros((1,n_mat+1), dtype=np.int32)))        
        
        #visualize matrix
        print('|', end = " ")
        for i in range(n_mat+1):
            if i == n_mat:
                print("sink", end=" ")
            elif i == n_mat-1:
                print("source", end=" ")
            else:
                print(f"node{i}", end=" ")
        print('|')
        for _, inner in enumerate(adj_matrix):
            print('|', end = " ")
            for val in inner:
                v = str(val)
                for i in range(5-len(v), 0, -1):
                    print(" ", end="")
                print(v, end=" ")
            print('|')

        return adj_matrix, remap_station_dict

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
        self.load_capacity = 4
        self.num_stops = 2

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

        station_list.sort()

        #construct vrpy here
        adj_matrix, remap_station_dict  = self.traverse_graph.create_adjacency_matrix(station_list) 

        self.adj_matrix = np.array(adj_matrix, dtype=[("cost", int)])
        self.vrp_graph = from_numpy_matrix(self.adj_matrix, create_using=nx.DiGraph())

        # The depot is relabeled as Source and Sink
        self.vrp_graph = relabel_nodes(self.vrp_graph, 
                                      {adj_matrix.shape[0]-2: "Source", #0: "Source", 
                                       adj_matrix.shape[0]-1: "Sink"})
 
        print("")
        print(f"nodes : {nx.nodes(self.vrp_graph)}")

        reverse_station_dict = {}
        for name, node in remap_station_dict.items():
            reverse_station_dict[node] = name

        for i, from_node in enumerate(pickups_list):
            from_node = remap_station_dict[from_node] #+ 1
            to_node = remap_station_dict[deliveries_list[i]] #+ 1
            demand = demands_list[i]
            self.vrp_graph.nodes[from_node]["request"] = to_node
            # Pickups are accounted for positively
            self.vrp_graph.nodes[from_node]["demand"] = demand
            # Deliveries are accounted for negatively
            self.vrp_graph.nodes[to_node]["demand"] = -demand

        print(f"demand : {nx.get_node_attributes(self.vrp_graph, 'demand')}")
        print(f"request : {nx.get_node_attributes(self.vrp_graph, 'request')}")
        print("")

        # nx.draw_shell(self.vrp_graph, with_labels=True)
        # plt.show()

        print("planning")
        prob = VehicleRoutingProblem(self.vrp_graph, 
                                     load_capacity=self.load_capacity,
                                     pickup_delivery=True,
                                     num_stops=self.num_stops)
        try:
            prob.solve(cspy=False, exact=True, pricing_strategy="BestPaths")
            routes = {}
            for i, path in prob.best_routes.items():
                print(f"path ({i}) :",end = " ")
                via = []
                for n in path:
                    if n in ["Source", "Sink"]:
                        print(f"depot({self.metadata.depot_node}) ||", end = " ")
                        via.append(self.metadata.depot_node)
                    else:
                        name = reverse_station_dict[n]
                        node = self.traverse_graph.stations[name]
                        print(f"{name}({node}) ||", end = " ")
                        via.append(node)
                routes[i] = via
                print("")
            self.info("plan !!success")

            self.send_user_mission(routes)
            return True

        except Exception as e:
            print(e)
            self.info("plan !!Failed")
            return False
        
    def request_map_metadata(self, time_out=5):
        self.info('wait for get map metdata service')
        get_map_client = self.create_client(GetMapMetadata, self.topic.get_map_metada)
        srv_ready = get_map_client.wait_for_service(timeout_sec=time_out)
        if not srv_ready:
            self.error('get map metdata service not ready')
            return False

        self.info('receive response from server')
        future = get_map_client.call_async(GetMapMetadata.Request())
        rclpy.spin_until_future_complete(self, future)

        self.metadata = future.result().metadata

        self.traverse_graph.set_map_metadata(self.metadata)
        self.traverse_graph.create_adjacency_dict() #hash fully connected all station

        return True

    def send_user_mission(self, routes):
        success_list = []
        for i, path in routes.items():
            user_mission_topic = f"/robot{i}/do_usermission"

            user_mission_client = self.create_client(UserMission, user_mission_topic)
            srv_ready = user_mission_client.wait_for_service(timeout_sec=3)
            if not srv_ready:
                self.error(f'{user_mission_topic} service not ready')
                success_list.append(False)
                continue

            self.info('receive response from server')

            req = UserMission.Request()
            print(path)
            req.user_mission.node_list = path

            future = user_mission_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.info(f"receive response from adapter : {future.result().success}")

            success_list.append(future.result().success)

        print(success_list)

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
    pickups_list =    ['A', 'B',  'F',  'H']
    deliveries_list = ['D', 'C',  'G',  'I']
    demand_list =     [ 1 ,  1,    1 ,   1 ]

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
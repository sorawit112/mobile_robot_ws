import rclpy
from rclpy.node import Node

from mission_manager.mission_executor import Unit
from visualization_msgs.msg import MarkerArray, Marker
from custom_msgs.msg import MapMetadata, MapNode, MapEdge, MapStation
from custom_msgs.srv import GetMapMetadata
from geometry_msgs.msg import Point
import os, glob, pathlib
import json
import math

class GraphLoader(Node):
    # Fleet Management Initialized This Node

    def __init__(self, map_name):
        super().__init__('graph_loader')

        self.map_name = map_name
        self._nodes, self._edges, self._edges_unit, self._stations = self.load_metadata(map_name)

        self.visualize_pub = self.create_publisher(MarkerArray, 'visualize_graph', qos_profile=1)
        self.create_service(GetMapMetadata, 'get_map_metadata', self.handle_get_metadata)
        self.create_timer(10, self.draw_graph_cb)

        self.get_logger().info('initial complete')

    def handle_get_metadata(self, request, response):

        self.get_logger().info('receive request cmd')

        map_metadata = MapMetadata()

        node_list = []
        for id, p in self._nodes.items():
            node = MapNode()
            node.id = id
            node.point = Point(x=p[0], y=p[1], z=0.)
            node_list.append(node)
           
        edge_list = []
        for id, d in self._edges.items():
            edge = MapEdge()
            edge.id = id
            edge.src = d[0]
            edge.dst = d[1]
            edge.weight = euc2d(self._nodes[d[0]],self._nodes[d[1]])
            edge.unit = self._edges_unit[id]
            edge_list.append(edge)

        station_list = []
        
        for name, node_id in self._stations.items():
            station = MapStation()
            station.name = name
            station.node_id = node_id
            station_list.append(station)

        map_metadata.nodes = node_list
        map_metadata.edges = edge_list
        map_metadata.stations = station_list

        response.metadata = map_metadata

        self.get_logger().info('response success')

        return response

    def load_metadata(self, map_name):
        # path = pathlib.Path(__file__).parent.resolve()
        # path = os.path.dirname(path)
        file = glob.glob('/home/trainee/ros2_ws/mobile_robot_ws/src/utility/map_data/map_data/' + str(self.map_name) + '/metadata.json')
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
            dict_edges_task[int(e)] = Unit._dict.value[str(d[1])]
        for s,n in data['stations'].items():
            dict_stations[str(s)] = int(n)

        return dict_nodes, dict_edges, dict_edges_task, dict_stations

    def draw_graph_cb(self):
        marker_list = MarkerArray()
        # draw nodes
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nodes"
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        marker.points.clear()
        text_id = 10
        station_name_list = list(self._stations.keys())
        station_node_list = list(self._stations.values())
        for n,p in self._nodes.items():
            marker.points.append(Point(x=p[0], y=p[1], z=0.))

            marker_text = Marker()
            marker_text.header.frame_id = 'map'
            marker_text.header.stamp = self.get_clock().now().to_msg()
            marker_text.ns = "node_name"
            marker_text.id = text_id
            marker_text.type = Marker.TEXT_VIEW_FACING
            marker_text.action = Marker.ADD
            marker_text.scale.x = 0.2
            marker_text.scale.y = 0.2
            marker_text.scale.z = 0.3
            marker_text.color.a = 1.0
            marker_text.color.r = 0.0
            marker_text.color.g = 0.0
            marker_text.color.b = 0.0
            marker_text.pose.position.x = p[0]-0.17
            marker_text.pose.position.y = p[1]-0.17
            marker_text.pose.position.z = 0.
            marker_text.pose.orientation.x = 0.
            marker_text.pose.orientation.y = 0.
            marker_text.pose.orientation.z = 0.
            marker_text.pose.orientation.w = 1.

            if n in station_node_list:
                marker_text.text = station_name_list[station_node_list.index(n)]
            else:
                marker_text.text = str(n)

            marker_list.markers.append(marker_text)
            text_id += 1

        marker_list.markers.append(marker)

        #draw edges
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "edges"
        marker.id = 2
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.05
        marker.scale.y = 0.
        marker.scale.z = 0.
        marker.color.a = 1.0
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5

        marker.points.clear()
        for e,d in self._edges.items():
            node_1 = self._nodes[d[0]]
            node_2 = self._nodes[d[1]]
            marker.points.append(Point(x=node_1[0], y=node_1[1], z=0.))
            marker.points.append(Point(x=node_2[0], y=node_2[1], z=0.))

        marker_list.markers.append(marker)

        self.visualize_pub.publish(marker_list)


def euc2d(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)    


def main(args=None):
    rclpy.init(args=args)
    MAP_NAME = os.environ['MAP_NAME']
    action_client = GraphLoader("map_"+str(MAP_NAME).lower())

    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
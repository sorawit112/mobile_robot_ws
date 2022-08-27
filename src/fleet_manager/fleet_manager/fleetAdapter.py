import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import os
from enum import Enum

from mission_manager.topics import Topics
from mission_manager.transform_manager import TransformManager
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from visualization_msgs.msg import MarkerArray, Marker

from custom_msgs.msg import UserMission as dotask
from custom_msgs.msg import Viapoint
from custom_msgs.srv import UserMission

ROBOT_NAME = os.environ['ROBOT_NAME']

class FleetAdapter(Node):
    def __init__(self):
        super().__init__(node_name="fleet_adapter")
        self.module_name = "fleet_adapter"

        self.current_pose = PoseStamped()
        self.adapter_interval = 0.5
        self.status = 0 
        self.count = 0
        """
            WAIT = 0
            NAVIGATE = 1
            ERROR = 2
        """

        self.topic = Topics()
        self.tf_manager = TransformManager(self)

        self.via_x = []
        self.via_y = []
        self.via_points_list = MarkerArray()

        self.visualize_pub = self.create_publisher(MarkerArray, self.topic.viz_viapoints, qos_profile=1)
        # self.status_pub = self.create_publisher(Int16, self.topic.status, qos_profile=1)
        self.pose_pub = self.create_publisher(PoseStamped, self.topic.current_pose, qos_profile=1)
        self.do_task_pub = self.create_publisher(dotask, self.topic.do_task, qos_profile=1)

        self.create_subscription(PoseWithCovarianceStamped, self.topic.amcl_pose, self.amcl_pose_cb, qos_profile=1)
        # self.create_subscription(Int16, self.topic.robot_status, self.robot_status_cb, qos_profile=1)
        self.create_subscription(Viapoint, self.topic.current_viapoints, self.current_viapoints_cb, qos_profile=1)
        self.create_service(UserMission, self.topic.do_usermission, self.do_usermission_cb)
        self.create_timer(self.adapter_interval, self.main_routine)

        self.info('initialize complete')

    def main_routine(self):
        self.info("{} pose: x{:.2f}, y{:.2f}".format(ROBOT_NAME,
                                                    self.current_pose.pose.position.x,
                                                    self.current_pose.pose.position.y))
        # self.info(f"{ROBOT_NAME} status: {self.status}")
        
        self.pose_pub.publish(self.current_pose)

        if self.count != 6:
            self.count = self.count+1
            return

        self.count = 0 

        n = len(self.via_x)
        if n > 1:
            #draw mission plan
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "/"+ROBOT_NAME+"/path"
            marker.id = 0
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD

            marker.scale.x = 0.02
            marker.scale.y = 0.
            marker.scale.z = 0.
            marker.color.a = 1.0

            marker.color.r = 0.
            marker.color.g = 0.
            marker.color.b = 0.

            if ROBOT_NAME == 'robot1':
                marker.color.r = 1.0
            elif ROBOT_NAME == 'robot2':
                marker.color.g = 1.0
            else:
                marker.color.b = 1.0

            marker.points.clear()
            
            for i in range(n-1):
                marker.points.append(Point(x=self.via_x[i], y=self.via_y[i], z=0.))
                marker.points.append(Point(x=self.via_x[i+1], y=self.via_y[i+1], z=0.))

            self.via_points_list.markers.append(marker)
            self.visualize_pub.publish(self.via_points_list)

    def amcl_pose_cb(self, msg):
        self.debug("receive amcl pose")
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose

    def do_usermission_cb(self, request, response):
        self.info("------------receive do_usermission request------------")
        node_list = request.user_mission.node_list
        metadata = request.map_metadata
        print(node_list)
        dotask_msg = dotask()
        dotask_msg.node_list = node_list
        dotask_msg.map_metadata = metadata
        
        self.do_task_pub.publish(dotask_msg)

        response.success = True
        return response

    # def robot_status_cb(self, msg):
    #     self.info(f"receive robot status : {msg.data}")
    #     self.status = msg.data

    def current_viapoints_cb(self, msg):
        self.info(f"receive current viapoints: {len(msg.x)} points")

        self.via_x = msg.x
        self.via_y = msg.y           

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
    fleet_adapter = FleetAdapter()

    try:
        rclpy.spin(fleet_adapter)
    except:
        fleet_adapter.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()


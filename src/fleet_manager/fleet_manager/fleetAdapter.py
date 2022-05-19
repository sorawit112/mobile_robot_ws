
from platform import node
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import os
from enum import Enum

from mission_manager.topics import Topics
from mission_manager.transform_manager import TransformManager
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from custom_msgs.msg import UserMission as dotask
from custom_msgs.srv import UserMission

ROBOT_NAME = os.environ['ROBOT_NAME']

class FleetAdapter(Node):
    def __init__(self, service_client):
        super().__init__(node_name="fleet_adapter")
        self.module_name = "fleet_adapter"

        self.current_pose = PoseStamped()
        self.adapter_interval = 0.5
        self.status = 0 
        """
            WAIT = 0
            NAVIGATE = 1
            ERROR = 2
        """

        self.topic = Topics()
        self.tf_manager = TransformManager(self)
        self.service_client = service_client

        self.status_pub = self.create_publisher(Int16, self.topic.status, qos_profile=1)
        self.pose_pub = self.create_publisher(PoseStamped, self.topic.current_pose, qos_profile=1)
        self.do_task_pub = self.create_publisher(dotask, self.topic.do_task, qos_profile=1)

        self.create_subscription(PoseWithCovarianceStamped, self.topic.amcl_pose, self.amcl_pose_cb, qos_profile=1)
        self.create_subscription(Int16, self.topic.robot_status, self.robot_status_cb, qos_profile=1)
        self.create_service(UserMission, self.topic.do_usermission, self.do_usermission_cb)
        self.create_timer(self.adapter_interval, self.main_routine)


    def main_routine(self):
        self.info("current_pose: x{:.2f}, y{:.2f}".format(self.current_pose.pose.position.x,
                                                    self.current_pose.pose.position.y))
        self.info(f"status: {self.status}")
        
        # current_tf, success = self.tf_manager.get_tf("map", "base_footprint")
        # if success:
        #     self.current_pose = self.tf_manager.pose_stamped_from_tf_stamped(current_tf)
        #     self.pose_pub.publish(self.current_pose)
        # else:
        #     self.warn("skip publish current_pose this frame")

        status_msg = Int16()
        status_msg.data = self.status
        self.status_pub.publish(status_msg)

    def amcl_pose_cb(self, msg):
        self.debug("receive amcl pose")
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose


    def do_usermission_cb(self, request, response):
        self.info("------------receive do_usermission request------------")
        node_list = request.user_mission.node_list
        print(node_list)
        dotask_msg = dotask()
        dotask_msg.node_list = node_list

        # self.service_client.call_do_task(node_list)
        self.do_task_pub.publish(dotask_msg)

        response.success = True
        return response

    def robot_status_cb(self, msg):
        self.info(f"receive robot status : {msg.data}")
        self.status = msg.data

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


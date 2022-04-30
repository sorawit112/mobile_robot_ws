import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import math
from enum import Enum

from mission_manager.transform_manager import TransformManager

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from custom_msgs.action import NavigateAction


class Unit(Enum):
    """ units instance list"""
    IDLE = 0
    NAV = 1
    DOCK = 2
    TURTLE = 3

    _dict = {'IDLE':IDLE, 'NAV':NAV, 'DOCK':DOCK, 'TURTLE':TURTLE}


class MissionUnitClient(object):
    def __init__(self,
                 node, 
                 unit_name, 
                 unit_id, 
                 action_spec, 
                 ):

        self.module_name = "unit_client"
        self.node = node
        self.unit_name = unit_name
        self.action_spec = action_spec
        self.unit_id = unit_id
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        self.action_topic = node.topic.robot_name + "/" + str(self.unit_name) + "_server"
        self.action_client = ActionClient(node, self.action_spec, self.action_topic)

    def is_server_ready(self):
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.do_logging("{self.action_topic} not available, waiting...")
        
        self.do_logging("{self.action_topic} available and ready to receive goal")
        
    def cancel_goal(self):
        """Cancel pending task request of any type."""
        self.do_logging('Canceling {self.unit_name} task')

        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
            self.result_future = future

            self.do_logging('Goal successfully canceled')

        return True

    def send_goal(self, goal_msg):
        self.do_logging('send goal to ' + str(self.action_topic))

        self.is_server_ready()

        send_goal_future = self.action_client.send_goal_async(goal_msg,
                                                              self._feedbackCallback)

        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.do_logging('Goal rejected :(')
            return False
        
        self.do_logging('Goal accepted :)')

        self.result_future = self.goal_handle.get_result_async()
        return True

    def is_task_complete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True

        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.do_logging('Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def do_logging(self, msg):
        self.node.get_logger().info("[{0}/{1}] {2}".format(self.module_name, self.unit_name, msg))

NODE_NAME = 'mission_executor'

class MissionExecutor(Node):
    
    def __init__(self, start_pose):
        super().__init__(node_name=NODE_NAME)
        self.module_name = NODE_NAME
        self.unit_dict = {}
        self.current_unit = None

        self.start_pose = start_pose

        self.create_mission_units()
        
    def destroyNode(self):
        """Destroy all units action_client and own node"""
        unit_list = list(self.unit_dict.values())
        for unit_client in unit_list:
            unit_client.action_client.destroy()
        super().destroy_node()

    def request_unit_execute(self, unit, task, last_pose):
        """ Requst input unit to execute"""
        navigate_goal = self.create_navigate_action_goal(task, last_pose)
        self.current_unit = self.unit_dict[unit]
        
        self.do_logging("request '{}' to execte".format(
                                                    self.current_unit.unit_name))

        self.log_unit_action_goal(navigate_goal)

        return self.current_unit.send_goal(navigate_goal)

    def request_current_unit_cancel(self):
        """Cancel current unit executing task"""
        self.do_logging("request '{}' to cancel".format(
                                                    self.current_unit.unit_name))
        return self.current_unit.cancel_goal()

    def is_unit_complete(self):
        """Check if the current unit executing is complete yet."""
        return self.current_unit.is_task_complete()

    def get_unit_result(self):
        """Get the current unit executing result message."""
        return self.current_unit.result_future.result().result
    
    def get_unit_status(self):
        """Get the current unit executing status message."""
        return self.current_unit.status

    def get_unit_feedback(self):
        """Get the current unit executing feedback message."""
        return self.current_unit.feedback
    
    def active_idle_unit(self, last_pose):
        idle_unit = self.unit_dict[Unit.IDLE]
        self.current_unit = idle_unit

        self.do_logging("active idle unit")
        navigate_goal = NavigateAction.Goal()
        navigate_goal.src_pose = last_pose
        navigate_goal.src_pose.header.stamp = self.get_clock().now().to_msg()
        
        idle_unit.send_goal(navigate_goal)

    def initial_first_unit(self):
        initial_unit = self.unit_dict[Unit.IDLE]
        self.current_unit = initial_unit

        server_ready = initial_unit.is_server_ready()

        if not server_ready:
            self.do_logging("initial first unit FAILED!!!")
            return
        
        self.do_logging("initial first unit")
        navigate_goal = NavigateAction.Goal()
        navigate_goal.src_pose.header.stamp = self.get_clock().now().to_msg()
        navigate_goal.src_pose.header.frame_id = "map"
        navigate_goal.src_pose.pose.position.x = self.start_pose[0]
        navigate_goal.src_pose.pose.position.y = self.start_pose[1]

        x,y,z,w = TransformManager.quaternion_from_euler(0., 0., self.start_pose[2])
        navigate_goal.src_pose.pose.orientation.x = x
        navigate_goal.src_pose.pose.orientation.y = y
        navigate_goal.src_pose.pose.orientation.z = z
        navigate_goal.src_pose.pose.orientation.w = w

        initial_unit.send_goal(navigate_goal)

    def create_mission_units(self):
        idle_unit = MissionUnitClient(
                            self,
                            "idle_unit",
                            unit_id=Unit.IDLE,
                            action_spec=NavigateAction
                            )

        nav_unit = MissionUnitClient(
                            self,
                            "nav_unit",
                            unit_id=Unit.NAV,
                            action_spec=NavigateAction
                            )

        docking_unit = MissionUnitClient(
                            self,
                            "docking_unit",
                            unit_id=Unit.DOCK,
                            action_spec=NavigateAction
                            )

        turtle_unit = MissionUnitClient(self,
                            "turtle_unit",
                            unit_id=Unit.TURTLE,
                            action_spec=NavigateAction
                            )

        self.unit_dict[Unit.IDLE] = idle_unit
        self.unit_dict[Unit.NAV] = nav_unit
        self.unit_dict[Unit.DOCK] = docking_unit
        self.unit_dict[Unit.TURTLE] = turtle_unit

    def create_navigate_action_goal(self, task, src_pose):
        goal = NavigateAction.Goal()
        goal.src_node = task.src_node
        goal.dst_node = task.dst_node

        src_pose.header.stamp = self.get_clock().now().to_msg()
        goal.src_pose = src_pose
        
        dst_pose = PoseStamped()
        dst_pose.header.stamp = self.get_clock().now().to_msg()
        dst_pose.header.frame_id = "map"
        dst_pose.pose.position.x = task.dst_pose[0]
        dst_pose.pose.position.y = task.dst_pose[1]
        dst_pose.pose.position.z = 0.0

        goal.dst_pose = self.calculate_heading(src_pose, dst_pose)
        goal.via_points = task.via_points

        return goal

    def calculate_heading(self, src_pose, dst_pose):
        y_diff = dst_pose.pose.position.y - dst_pose.pose.position.y
        x_diff = src_pose.pose.position.x - src_pose.pose.position.x

        theta = math.atan2(y_diff, x_diff)

        x,y,z,w = TransformManager.quaternion_from_euler(0,0,theta)

        dst_pose.pose.orientation.x = x
        dst_pose.pose.orientation.y = y
        dst_pose.pose.orientation.z = z
        dst_pose.pose.orientation.w = w

        return dst_pose

    def log_unit_action_goal(self, goal):
        self.do_logging("FROM : [{}] ({}, {})".format(
            goal.src_node,
            goal.src_pose.pose.position.x,
            goal.src_pose.pose.position.y,
        ))

        self.do_logging("TO : [{}] ({}, {})".format(
            goal.dst_node,
            goal.dst_pose.pose.position.x,
            goal.dst_pose.pose.position.y,
        ))

    def do_logging(self, msg):
        self.get_logger().info("[{0}] {1}".format(self.module_name, msg))
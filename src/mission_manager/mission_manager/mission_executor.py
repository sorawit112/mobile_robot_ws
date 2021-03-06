import rclpy
from rclpy.node import Node
from rclpy.executors import Executor
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import math
import time

from enum import Enum

from mission_manager.topics import Topics

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


from lifecycle_msgs.srv import GetState
from std_srvs.srv import Trigger
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes

from nav2_msgs.action import FollowWaypoints, NavigateToPose, Spin

class Unit(Enum):
    """ units instance list"""
    NAV = 1
    DOCK = 2
    TURTLE = 3

    _dict = {'NAV':NAV, 'DOCK':DOCK, 'TURTLE':TURTLE}


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class MissionExecutor(Node):

    def __init__(self):
        super().__init__(node_name='mission_executor')
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.latest_pose = PoseStamped()
        self.topic = Topics()

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.initial_pose_received = False
        
        # Nav2 Action Client
        self.nav_to_pose_client = ActionClient(self, 
                                               NavigateToPose, 
                                               'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, 
                                                    FollowWaypoints, 
                                                    'follow_waypoints')
        self.spin_client = ActionClient(self, 
                                        Spin, 
                                        'spin')
        
        # Custom Action Client
        self.docking_client = ActionClient(self, 
                                           NavigateToPose, 
                                           self.topic.robot_name+'/docking_unit_server')
        self.turtle_client = ActionClient(self, 
                                           NavigateToPose, 
                                           self.topic.robot_name+'/turtle_unit_server')
    

        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        self.change_maps_srv = self.create_client(LoadMap, '/map_server/load_map')
        self.clear_costmap_global_srv = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_costmap_local_srv = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.get_costmap_global_srv = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        self.get_costmap_local_srv = self.create_client(GetCostmap, '/local_costmap/get_costmap')

    def destroyNode(self):
        self.nav_to_pose_client.destroy()
        self.follow_waypoints_client.destroy()
        self.spin_client.destroy()
        self.docking_client.destroy()
        self.turtle_client.destroy()
        super().destroy_node()

    def setInitialPose(self, initial_pose):
        """Set the initial pose to the localization system."""
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self.initial_pose.header.stamp = self.get_clock().now().to_msg()
        self._setInitialPose()

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def followWaypoints(self, poses):
        """Send a `FollowWaypoints` action request."""
        self.debug("Waiting for 'FollowWaypoints' action server")
        while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.info(f'Following {len(goal_msg.poses)} goals....')
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg,
                                                                        self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(f'Following {len(poses)} waypoints request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def spin(self, spin_dist=1.57):
        self.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist

        self.info(f'Spinning to angle {goal_msg.target_yaw}....')
        send_goal_future = self.spin_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def goDocking(self, pose):
        """Send a `Docking` action request."""
        self.debug("Waiting for 'Docking' action server")
        while not self.docking_client.wait_for_server(timeout_sec=1.0):
            self.info("'Docking' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = ''

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.docking_client.send_goal_async(goal_msg,
                                                               self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def goTurtle(self, pose):
        """Send a `Turtle Control` action request."""
        self.debug("Waiting for 'Turtle Control' action server")
        while not self.turtle_client.wait_for_server(timeout_sec=1.0):
            self.info("'Turtle Control' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = ''

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.turtle_client.send_goal_async(goal_msg,
                                                              self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                        str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelTask(self):
        """Cancel pending task request of any type."""
        self.info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Task succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback
    
    def reset_module(self):
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate(localizer)
        if localizer == 'amcl':
            self._waitForInitialPose()
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return

    def pause_localizer(self):
        # check is localizer is pause
        check_active_srv_name = '/lifecycle_manager_localization/is_active'
        manage_node_srv_name = '/lifecycle_manager_localization/manage_nodes'
        self.info(f'Pause {check_active_srv_name}')
        if not self._isLifeCycleActive(check_active_srv_name):
            self.info('amcl is not active yet -> return')
            return

        self.manage_lifecycle_node(manage_node_srv_name, ManageLifecycleNodes.Request.PAUSE)

    def resume_localizer(self):
        # check is localizer is pause
        check_active_srv_name = '/lifecycle_manager_localization/is_active'
        manage_node_srv_name = '/lifecycle_manager_localization/manage_nodes'
        self.info(f'Resume {check_active_srv_name}')
        if self._isLifeCycleActive(check_active_srv_name):
            self.info('amcl is already active')
            return

        self.manage_lifecycle_node(manage_node_srv_name, ManageLifecycleNodes.Request.RESUME)   
        self._waitForNodeToActivate('amcl')

    def pause_navigator(self):
        # check is localizer is pause
        check_active_srv_name = '/lifecycle_manager_navigation/is_active'
        manage_node_srv_name = '/lifecycle_manager_navigation/manage_nodes'
        self.info(f'Pause {check_active_srv_name}')
        if not self._isLifeCycleActive(check_active_srv_name):
            self.info('navigator is not active yet -> return')
            return

        self.manage_lifecycle_node(manage_node_srv_name, ManageLifecycleNodes.Request.PAUSE)

    def resume_navigator(self):
        # check is localizer is pause
        check_active_srv_name = '/lifecycle_manager_navigation/is_active'
        manage_node_srv_name = '/lifecycle_manager_navigation/manage_nodes'
        self.info(f'Resume {check_active_srv_name}')
        if self._isLifeCycleActive(check_active_srv_name):
            self.info('navigator is already active')
            return 

        self.manage_lifecycle_node(manage_node_srv_name, ManageLifecycleNodes.Request.RESUME)
        self._waitForNodeToActivate('bt_navigator')

    def manage_lifecycle_node(self, srv_name, cmd):
        self.info('Managelifecycle nodes based on lifecycle_manager.')
        self.info(f'{srv_name}')

        mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{srv_name} service not available, waiting...')
        req = ManageLifecycleNodes.Request()
        req.command = cmd
        future = mgr_client.call_async(req)

        while True:
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.10)
            if future.result() is not None:
                self.info(f'mgr response : {future.result().success}')
                return future.result().success
            else:
                self.info('waitting mgr response')

    def lifecycleStartup(self):
        """Startup nav2 lifecycle system."""
        self.info('Starting up lifecycle nodes based on lifecycle_manager.')
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info(f'Starting up {srv_name}')
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f'{srv_name} service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request.STARTUP
                future = mgr_client.call_async(req)

                # starting up requires a full map->odom->base_link TF tree
                # so if we're not successful, try forwarding the initial pose
                while True:
                    rclpy.spin_until_future_complete(self, future, timeout_sec=0.10)
                    if not future:
                        self._waitForInitialPose()
                    else:
                        break
        self.info('Nav2 is ready for use!')
        return

    def lifecycleShutdown(self):
        """Shutdown nav2 lifecycle system."""
        self.info('Shutting down lifecycle nodes based on lifecycle_manager.')
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info(f'Shutting down {srv_name}')
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f'{srv_name} service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request.SHUTDOWN
                future = mgr_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                future.result()
        return

    def _isLifeCycleActive(self, srv_name):
        state_client = self.create_client(Trigger, srv_name)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{srv_name} service not available, waiting...')
        self.info(f'{srv_name} is ready')

        req = Trigger.Request()
        self.debug(f'Getting {srv_name} state...')
        future = state_client.call_async(req)
        while True:
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if future.result() is not None:
                return future.result().success   
            else:
                self.info(f'timeout call service {srv_name}')

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
        return

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        self.latest_pose.pose = msg.pose.pose
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

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
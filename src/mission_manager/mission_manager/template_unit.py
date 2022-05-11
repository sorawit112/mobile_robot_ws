import threading
from mission_manager.topics import Topics
from mission_manager.transform_manager import TransformManager

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

class TemplateUnit(Node):
    def __init__(self, node_name, loop_frequency, action_spec, execute_cb=None, 
                                                                goal_cb=None, 
                                                                accepted_cb=None, 
                                                                cancel_cb=None):
        super().__init__(node_name)
        self.node_name = node_name
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        
        self.topic = Topics()
        self.tf_manager = TransformManager(self)
        self.base_tf = self.tf_manager.create_tf('map', self.topic.base_footprint)
        self.odom_tf = self.tf_manager.create_tf('map', self.topic.odom)

        self.working = False
        self.rate = self.create_rate(loop_frequency)

        _execute_callback = execute_cb if execute_cb != None else self.default_execute_cb
        _goal_callback = goal_cb if goal_cb != None else self.default_goal_cb
        _accepted_callback = accepted_cb if accepted_cb != None else self.default_accepted_cb
        _cancel_callback = cancel_cb if cancel_cb != None else self.default_cancel_cb

        self._action_server = ActionServer( self,
                                            action_spec,
                                            self.topic.robot_name + '/'+ str(self.node_name) + '_server',
                                            execute_callback=_execute_callback,
                                            goal_callback=_goal_callback,
                                            handle_accepted_callback=_accepted_callback,
                                            cancel_callback=_cancel_callback)
    
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def default_execute_cb(self):
        """Execute the goal."""
        self.do_logging('execute cb must be implemented in worker package')
        raise NotImplementedError

    def default_goal_cb(self, goal_handle):
        """Accept or reject a client request to begin an action."""
        self.do_logging('Received goal request')
        return GoalResponse.ACCEPT

    def default_accepted_cb(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.do_logging('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def default_cancel_cb(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.do_logging('Received cancel request')
        return CancelResponse.ACCEPT

    def publish_tf(self,tf=None, delay=0):
        if tf is None:
            tf = self.base_tf
        self.tf_manager.publish_tf(tf, delay)

    def publish_odom_tf(self, robot_pose_kdl=None, stamp=None, delay=0):
        if robot_pose_kdl is None:
            robot_pose_kdl = self.tf_manager.transform_to_kdl(self.base_tf)
        self.tf_manager.publish_odom_tf(robot_pose_kdl, stamp, delay)   

    def get_tf(self, src_frame, target_frame, stamp=None):
        tf, result = self.tf_manager.get_tf(src_frame, target_frame, stamp)
        return tf, result

    def do_logging(self, msg):
        self.get_logger().info("[{}] {}".format(self.node_name, msg))
import threading

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

class TemplateWorker(Node):
    def __init__(self, node_name, action_spec, execute_cb=None, 
                                               goal_cb=None, 
                                               accepted_cb=None, 
                                               cancel_cb=None):
        super().__init__(node_name)
        self.node_name = node_name
        self._goal_handle = None
        self._goal_lock = threading.Lock()

        _execute_callback = execute_cb if execute_cb != None else self.default_execute_cb
        _goal_callback = goal_cb if goal_cb != None else self.default_goal_cb
        _accepted_callback = accepted_cb if accepted_cb != None else self.default_accepted_cb
        _cancel_callback = cancel_cb if cancel_cb != None else self.default_cancel_cb

        self._action_server = ActionServer( self,
                                            action_spec,
                                            '/'+str(self.node_name)+'_server',
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

    def do_logging(self, msg):
        self.get_logger().info("[{}] {}".format(self.node_name, msg))
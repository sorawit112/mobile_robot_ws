import rclpy
import time
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci

NODE_NAME = 'docking_worker'
class DockingWorker(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        self.node_name = NODE_NAME
        self._action_server = ActionServer(
            self,
            Fibonacci,
            '/'+str(self.node_name)+'_server',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.do_logging('Executing goal...')
        self.do_logging('receive request goal : {}'.format(goal_handle.request.order))
        goal = goal_handle.request.order
        self.do_logging('receive request goal : {}'.format(goal))
        time.sleep(goal)
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = goal_handle.request.order
        return result

    def do_logging(self, msg):
        self.get_logger().info("[{}] {}".format(self.node_name, msg))


def main(args=None):
    rclpy.init(args=args)

    idle_worker = DockingWorker()

    rclpy.spin(idle_worker)


if __name__ == '__main__':
    main()
import time

from custom_msgs.action import NavigateAction
from task_manager.template_unit import TemplateUnit
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.executors import MultiThreadedExecutor

NODE_NAME = 'docking_unit'
class DockingUnit(TemplateUnit):
    frequency = 10
    def __init__(self):
        super().__init__(NODE_NAME, self.frequency, NavigateAction, self.execute_callback)
        self.controller = None
        
    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.do_logging('Executing goal...')

        #Initialze feedback msg
        feedback_msg = NavigateAction.Feedback()

        self.controller.active = True

        # Start executing the action
        while not self.controller.goal_reach:
            # If goal is flagged as no longer active (ie. another goal was accepted),then stop executing
            if not goal_handle.is_active:
                goal_handle.aborted()
                self.do_logging('Goal aborted')
                return NavigateAction.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.do_logging('Goal canceled')
                return NavigateAction.Result()

            self.controller.control_loop()

            # Update Fibonacci sequence
            feedback_msg.current_pose = self.controller.current_pose

            self.do_logging('Publishing feedback: {0}'.format(feedback_msg.current_pose))

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            self.control_rate.sleep()

        goal_handle.succeed()

        # Populate result message
        result = NavigateAction.Result()
        result.last_pose = self.controller.current_pose
        
        self.do_logging('Returning result: {0}'.format(result.last_pose))

        return result


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DockingUnit()
        
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
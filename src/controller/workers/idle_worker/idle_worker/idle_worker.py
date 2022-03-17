import time

from action_tutorials_interfaces.action import Fibonacci
from task_manager.template_worker import TemplateWorker

import rclpy
from rclpy.executors import MultiThreadedExecutor

NODE_NAME = 'idle_worker'
class IdleWorker(TemplateWorker):
    def __init__(self):
        super().__init__(NODE_NAME, Fibonacci, self.execute_callback)
        
    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.do_logging('Executing goal...')

        # Append the seeds for the Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Start executing the action
        for i in range(1, goal_handle.request.order):
            # If goal is flagged as no longer active (ie. another goal was accepted),
            # then stop executing
            if not goal_handle.is_active:
                goal_handle.aborted()
                self.do_logging('Goal aborted')
                return Fibonacci.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.do_logging('Goal canceled')
                return Fibonacci.Result()

            # Update Fibonacci sequence
            feedback_msg.partial_sequence.append(feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            self.do_logging('Publishing feedback: {0}'.format(feedback_msg.partial_sequence))

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            time.sleep(1)

        goal_handle.succeed()

        # Populate result message
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence

        self.do_logging('Returning result: {0}'.format(result.sequence))

        return result


def main(args=None):
    rclpy.init(args=args)

    idle_worker = IdleWorker()

    executor = MultiThreadedExecutor()
    rclpy.spin(idle_worker, executor=executor)

    idle_worker.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
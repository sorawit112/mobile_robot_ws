import time

from custom_msgs.action import NavigateAction
from task_manager.template_worker import TemplateWorker
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

import rclpy
from rclpy.executors import MultiThreadedExecutor

NODE_NAME = 'idle_worker'
class IdleWorker(TemplateWorker):
    frequency = 1
    def __init__(self):
        super().__init__(NODE_NAME, self.frequency, NavigateAction, self.execute_callback)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.on_initial_pose, qos_profile=1)
        
    def on_initial_pose(self, msg):
        if msg.header.frame_id == 'map':
            pass
        else:
            pass

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.do_logging('Executing goal...')

        current_pose = goal_handle.request.src_pose
        feedback_msg = NavigateAction.Feedback()

        # Start executing the action
        while True: #infinite loop wait end when cancel request from clients

            # If goal is flagged as no longer active (ie. another goal was accepted),
            # then stop executing
            if not goal_handle.is_active:
                goal_handle.aborted()
                self.do_logging('Goal aborted')
                return NavigateAction.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.do_logging('Goal canceled')
                return NavigateAction.Result()

            # Update current pose
            self.worker_transform = self.get_tf()
            feedback_msg.current_pose = self.tfstamp_to_posestamp(self.worker_transform)

            self.do_logging('Publishing feedback: {0}'.format(feedback_msg.current_pose))

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    idle_worker = IdleWorker()

    executor = MultiThreadedExecutor()
    rclpy.spin(idle_worker, executor=executor)

    idle_worker.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
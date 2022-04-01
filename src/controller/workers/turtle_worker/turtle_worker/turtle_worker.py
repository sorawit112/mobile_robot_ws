import time
from turtle_worker.controller import Controller

from custom_msgs.action import NavigateAction
from task_manager.template_worker import TemplateWorker
from geometry_msgs.msg import PoseStamped, Pose2D

import rclpy
from rclpy.executors import MultiThreadedExecutor

NODE_NAME = 'turtle_worker'
class TurtleWorker(TemplateWorker):
    frequency = 10
    def __init__(self):
        super().__init__(NODE_NAME, self.frequency, NavigateAction, self.execute_callback)
        self.current_pose = PoseStamped()
        self.controller = Controller(self)
        
    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.do_logging('Executing goal...')

        feedback_msg = NavigateAction.Feedback()
        
        dst_pose = goal_handle.request.dst_pose
        src_pose = goal_handle.request.src_pose
        
        if dst_pose is None:
            return self.abort_handle(goal_handle, "dst_pose is None")

        self.working = True
        self.current_pose = src_pose
        self.initial_controller(src_pose, dst_pose)

        # intitial localization
        self.worker_transform, result = self.get_tf("map", "robot_pose")
        if not result:
            pass
            # return self.abort_handle(goal_handle, "can't get tf map->robot_pose")
        stamp = self.get_clock().now()
        self.publish_tf()

        # Start executing the action
        while not self.controller.goal_reach:
            if not goal_handle.is_active:
                return self.abort_handle(goal_handle, "goal not active")

            if goal_handle.is_cancel_requested:
                return self.preempt_handle(goal_handle)

            self.controller.control_loop()

            # update current map->robot_pose
            self.current_pose = self.controller.get_current_pose_stamped(self.current_pose)

            _, result = self.get_tf("map", "robot_pose", stamp) 
            self.worker_transform = self.tf_manager.tf_stamped_from_pose_stamped(self.current_pose, "robot_pose")
            stamp = self.get_clock().now()
            self.publish_tf()

            # Publish the feedback
            feedback_msg.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback_msg)
            self.do_logging('Publishing feedback: {0}'.format(feedback_msg.current_pose))

            self.rate.sleep() #declare in super

        # goal succeed
        return self.succes_handle(goal_handle)

    def initial_controller(self, src_pose, dst_pose):
        self.controller.goal_pose = Pose2D( x=dst_pose.pose.position.x,
                                            y=dst_pose.pose.position.y,
                                            theta=self.tf_manager.euler_from_quaternion(
                                                        dst_pose.pose.orientation.x,
                                                        dst_pose.pose.orientation.y,
                                                        dst_pose.pose.orientation.z,
                                                        dst_pose.pose.orientation.w))
        self.controller.current_pose = Pose2D( x=src_pose.pose.position.x,
                                               y=src_pose.pose.position.y,
                                               theta=self.tf_manager.euler_from_quaternion(
                                                        src_pose.pose.orientation.x,
                                                        src_pose.pose.orientation.y,
                                                        src_pose.pose.orientation.z,
                                                        src_pose.pose.orientation.w)) 

    def preexit_result(self):
        self.working = False

        result = NavigateAction.Result()
        result.last_pose = self.current_pose

        return result

    def preempt_handle(self, goal_handle):
        self.do_logging('Goal canceled')
        result = self.preexit_result()
        goal_handle.canceled()
        
        return result

    def abort_handle(self, goal_handle, msg=""):
        self.do_logging("Goal aborted by {msg}")
        result = self.preexit_result()
        goal_handle.aborted()

        return result

    def succes_handle(self, goal_handle):
        self.do_logging('Goal succeed')
        result = self.preexit_result()
        goal_handle.succeed()
        self.do_logging('Returning result: {0}'.format(result.last_pose))

        return result


def main(args=None):
    rclpy.init(args=args)

    turtle_worker = TurtleWorker()

    executor = MultiThreadedExecutor()
    rclpy.spin(turtle_worker, executor=executor)

    turtle_worker.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
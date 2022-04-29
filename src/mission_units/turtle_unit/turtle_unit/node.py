from turtle_unit.controller import Controller

from custom_msgs.action import NavigateAction
from task_manager.template_unit import TemplateUnit
from geometry_msgs.msg import PoseStamped, Pose2D

import rclpy
from rclpy.executors import MultiThreadedExecutor

NODE_NAME = 'turtle_unit'
class TurtleUnit(TemplateUnit):
    frequency = 10
    def __init__(self):
        super().__init__(NODE_NAME, self.frequency, NavigateAction, self.execute_callback)
        self.current_pose = PoseStamped()
        self.controller = Controller(self)

        self.do_logging("initial complete!!")
        
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
        self.base_tf, result = self.get_tf("map", self.topic.base_footprint)
        if not result:
            self.base_tf = self.tf_manager.tf_stamped_from_pose_stamped(self.current_pose, self.topic.base_footprint)
            self.do_logging("can't get tf map->base_footprint -> use tf from src_pose")
        
            # return self.abort_handle(goal_handle, "can't get tf map->base_footprint")

        self.odom_tf = self.tf_manager.odom_from_base_footprint(self.base_tf)
        self.publish_tf(tf=self.odom_tf)

        # Start executing the action
        while not self.controller.goal_reach:
            # get current_pose
            self.base_tf, result = self.get_tf("map", self.topic.base_footprint)
            if not result:
                return self.abort_handle(goal_handle, "can't get tf map -> base_footprint")

            if not goal_handle.is_active:
                return self.abort_handle(goal_handle, "goal not active")

            if goal_handle.is_cancel_requested:
                return self.preempt_handle(goal_handle)

            self.current_pose = self.tf_manager.pose_stamped_from_tf_stamped(self.base_tf)
            self.controller.current_pose2D_from_pose_stamped(self.current_pose)

            self.controller.control_loop()

            # Publish the feedback
            feedback_msg.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback_msg)
            self.do_logging('Publishing feedback: x: {0}, y: {1}'.format(self.current_pose.pose.position.x,
                                                                         self.current_pose.pose.position.y))      

            # publish map -> odom for next loop
            self.publish_tf(tf=self.odom_tf, delay=1.3/self.frequency)
            self.rate.sleep()

        # goal succeed
        return self.succes_handle(goal_handle)

    def initial_controller(self, src_pose, dst_pose):
        _, _, goal_theta = self.tf_manager.euler_from_quaternion(
                                                        dst_pose.pose.orientation.x,
                                                        dst_pose.pose.orientation.y,
                                                        dst_pose.pose.orientation.z,
                                                        dst_pose.pose.orientation.w)
        self.controller.goal_pose = Pose2D( x=dst_pose.pose.position.x,
                                            y=dst_pose.pose.position.y,
                                            theta=goal_theta)

        _, _, current_theta = self.tf_manager.euler_from_quaternion(
                                                        src_pose.pose.orientation.x,
                                                        src_pose.pose.orientation.y,
                                                        src_pose.pose.orientation.z,
                                                        src_pose.pose.orientation.w)
        self.controller.current_pose = Pose2D( x=src_pose.pose.position.x,
                                               y=src_pose.pose.position.y,
                                               theta=current_theta) 

    def preexit_result(self):
        self.working = False
        self.controller.goal_reach = False

        result = NavigateAction.Result()
        result.last_pose = self.current_pose

        return result

    def preempt_handle(self, goal_handle):
        self.do_logging('Goal canceled')
        result = self.preexit_result()
        goal_handle.canceled()
        
        return result

    def abort_handle(self, goal_handle, msg=""):
        self.do_logging("Goal aborted by {}".format(msg))
        result = self.preexit_result()
        goal_handle.abort()

        return result

    def succes_handle(self, goal_handle):
        self.do_logging('Goal succeed')
        result = self.preexit_result()
        goal_handle.succeed()
        self.do_logging('Returning result: {0}'.format(result.last_pose))

        return result


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TurtleUnit()
        
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
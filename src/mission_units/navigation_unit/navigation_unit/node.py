from re import S
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from custom_msgs.action import NavigateAction
from task_manager.template_unit import TemplateUnit
from geometry_msgs.msg import PoseStamped
from navigation_unit.robot_navigator import BasicNavigator, TaskResult

NODE_NAME = 'nav_unit'
TIME_OUT_NAVIGATION = 600

class NavigationUnit(TemplateUnit):
    frequency = 10
    def __init__(self):
        super().__init__(NODE_NAME, self.frequency, NavigateAction, self.execute_callback)
        self.current_pose = PoseStamped()
        self.navigator = BasicNavigator()
        self.initial_pose = PoseStamped()

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.do_logging('Executing goal...')

        feedback_msg = NavigateAction.Feedback()
        dst_pose = goal_handle.request.dst_pose
        src_pose = goal_handle.request.src_pose
        via_points = goal_handle.request.via_points
        
        if dst_pose is None and src_pose is None:
            return self.abort_handle(goal_handle, 'dst_pose or src_pose is None!!')

        self.working = True
        self.current_pose = src_pose

        self.initial_pose = src_pose
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        self.navigator.setInitialPose(self.initial_pose)

        self.navigator.lifecycleStartup()

        self.navigator.waitUntilNav2Active()

        goal_pose = dst_pose
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
    
        if len(via_points) == 0: # navigate to pose
            self.navigator.goToPose(goal_pose)
            nav_mode = 0

        else: #navigate follow via_points
            for pose in via_points:
                pose.header.stamp = self.navigator.get_clock().now().to_msg()

            via_points.append(goal_pose)
            self.navigator.followWaypoints(via_points)
            nav_mode = 1
        
        nav_start = self.navigator.get_clock().now()

        i = 1
        while not self.navigator.isTaskComplete():
            self.base_tf, result = self.get_tf("map", self.topic.base_footprint)
            if not result:
                return self.abort_handle(goal_handle, "can't get tf map -> base_footprint")

            self.current_pose = self.tf_manager.pose_stamped_from_tf_stamped(self.base_tf)
            
            # Publish the feedback to mission executer
            feedback_msg.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback_msg)
            self.do_logging('Publishing feedback: x: {0}, y: {1}'.format(self.current_pose.pose.position.x,
                                                                         self.current_pose.pose.position.y))      

            #get feedback from navigator
            nav_feedback = self.navigator.getFeedback()
            if nav_feedback and i % 5 == 0:
                if nav_mode == 0: # go_to_pose
                    self.do_logging('Estimated time of arrival:  + {0:.0f} seconds.'.format(
                        Duration.from_msg(nav_feedback.estimated_time_remaining).nanoseconds / 1e9))
                
                elif nav_mode == 1: # follow_via_points
                    self.do_logging('Executing current waypoint: {0} of {1}'.format(
                        str(nav_feedback.current_waypoint + 1), 
                        str(len(via_points))))

                nav_now = self.navigator.get_clock().now()
                
                if nav_now - nav_start > Duration(TIME_OUT_NAVIGATION):
                    self.navigator.cancelTask()
                    return self.abort_handle(goal_handle, 'navigator time out!!')
            
            if not goal_handle.is_active:
                return self.abort_handle(goal_handle)
            if goal_handle.is_cancel_requested:
                return self.preempt_handle(goal_handle)

            i += 1
            self.rate.sleep()

        self.base_tf, result = self.get_tf("map", self.topic.base_footprint)
        if result:
            self.current_pose = self.tf_manager.pose_stamped_from_tf_stamped(self.base_tf)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.succes_handle(goal_handle)
        elif result == TaskResult.CANCELED:
            self.abort_handle(goal_handle, 'navigator got cancel request!!')
        elif result == TaskResult.FAILED:
            self.abort_handle(goal_handle, 'navigator failed to navigate to dst_pose!!')
        else:
            self.abort_handle(goal_handle, 'Navigator Goal has an invalid return status!!')

    def preexit_result(self):
        self.working = False
        if not self.navigator.isTaskComplete(): #cancel navigator task if not complete
            self.navigator.cancelTask()
        self.navigator.lifecycleShutdown()
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
        node = NavigationUnit()
        
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.navigator.destroyNode()
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
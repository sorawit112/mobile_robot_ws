from custom_msgs.action import NavigateAction
from task_manager.template_worker import TemplateWorker
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped

import rclpy
from rclpy.executors import MultiThreadedExecutor

NODE_NAME = 'idle_worker'
class IdleWorker(TemplateWorker):
    frequency = 1
    def __init__(self):
        super().__init__(NODE_NAME, self.frequency, NavigateAction, self.execute_callback)
        self.current_pose = PoseStamped()
        self.new_worker_kdl = None

        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, qos_profile=1)
        
        self.do_logging("initial complete!!")

    def on_initial_pose(self, msg):
        frame_id = msg.header.frame_id
        self.do_logging("Initial pose received ! frame_id : {}".format(frame_id))

        if not self.working:
            self.do_logging("Not Active ! -> do nothing")
            return

        if 'map' in frame_id:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose
            self.worker_transform = self.tf_manager.tf_stamped_from_pose_stamped(pose_stamped, self.topic.base_footprint)
            self.odom_tf = self.tf_manager.odom_from_base_footprint(self.worker_transform)
            
            self.do_logging("receive new odom")

        else:
            self.do_logging('pass')

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.do_logging('Executing goal...')

        feedback_msg = NavigateAction.Feedback()

        self.current_pose = goal_handle.request.src_pose #pose_stamped
        if self.current_pose is None:
            return self.abort_handle(goal_handle, "src_pose is None")

        self.working = True

        self.worker_transform = self.tf_manager.tf_stamped_from_pose_stamped(self.current_pose, self.topic.base_footprint)
        self.odom_tf = self.tf_manager.odom_from_base_footprint(self.worker_transform)

        self.publish_tf(tf=self.odom_tf)
        
        # Start executing the action
        while True: #infinite loop until cancel request from clients
            if not goal_handle.is_active:
                return self.abort_handle(goal_handle, "goal not active")

            if goal_handle.is_cancel_requested:
                return self.preempt_handle(goal_handle)

            # get current_pose then publish map -> odom
            robot_pose_tf, result = self.get_tf("map", self.topic.base_footprint)
            if not result:
                return self.abort_handle(goal_handle, "can't get tf map -> base_footprint")

            # publish map -> odom for next loop
            self.publish_tf(tf=self.odom_tf, delay=1.1/self.frequency)

            # Publish the feedback
            self.current_pose = self.tf_manager.pose_stamped_from_tf_stamped(self.worker_transform)

            feedback_msg.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback_msg)
            self.do_logging('Publishing feedback : x:{0}, y:{1}'.format(
                                                        feedback_msg.current_pose.pose.position.x,
                                                        feedback_msg.current_pose.pose.position.y))

            self.rate.sleep() #declare in super
    
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
        self.do_logging("Goal aborted by {}".format(msg))
        result = self.preexit_result()
        goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)

    idle_worker = IdleWorker()

    executor = MultiThreadedExecutor(num_threads=5)
    rclpy.spin(idle_worker, executor=executor)

    idle_worker.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
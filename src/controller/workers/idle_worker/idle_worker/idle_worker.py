import time
from turtle import delay

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
        self.current_pose = PoseStamped()

        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, qos_profile=1)
        
    def on_initial_pose(self, msg):
        frame_id = msg.header.frame_id
        self.do_logging("Initial pose received ! frame_id : {frame_id}")

        if not self.working:
            self.do_logging("Not Active ! -> do nothing")
            return

        if 'map' in frame_id:
            self.worker_transform = self.tf_manager.tf_stamped_from_pose_stamped(msg.pose, 'robot_pose')
            self.publish_tf()

            # TODO use when odom fram ready
            # self.publish_odom_tf()
        else:
            pass

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.do_logging('Executing goal...')

        feedback_msg = NavigateAction.Feedback()

        self.current_pose = goal_handle.request.src_pose #pose_stamped
        if self.current_pose is None:
            return self.abort_handle(goal_handle, "src_pose is None")

        self.working = True

        self.worker_transform = self.tf_manager.tf_stamped_from_pose_stamped(self.current_pose, "robot_pose")
        stamp = self.get_clock().now()
        self.publish_tf() #publish map->robot_pose
        
        # Start executing the action
        while True: #infinite loop until cancel request from clients
            if not goal_handle.is_active:
                return self.abort_handle(goal_handle, "goal not active")

            if goal_handle.is_cancel_requested:
                return self.preempt_handle(goal_handle)

            # TODO: uncommend when odom frame is ready
            # publish map -> odom 
            # odom_tf, result = self.get_tf("map", "odom")
            # if not result:
            #     continue
            # self.publish_tf(tf=odom_tf)

            # get current_pose then publish map -> robot_pose 
            # TODO: use base_footprint instead of robot_pose when map->odom->base_footprint is ready
            robot_pose_tf, result = self.get_tf("map", "robot_pose", stamp=stamp)
            if result:
                self.worker_transform = robot_pose_tf

            stamp = self.get_clock().now()
            self.publish_tf() # Maintain tf to next loop

            # Publish the feedback
            current_pose = self.tf_manager.pose_stamped_from_tf_stamped(self.worker_transform)
            print(self.worker_transform.header.frame_id, self.worker_transform.child_frame_id)

            feedback_msg.current_pose = current_pose
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
        self.do_logging("Goal aborted by {msg}")
        result = self.preexit_result()
        goal_handle.aborted()

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
from geometry_msgs.msg import Pose2D, Twist
from turtlesim.msg import Pose
import numpy as np
import math


class Controller(object):
    k_v = 0.2
    k_w = 0.5

    def __init__(self, worker):
        self.module_name = "controller"
        self.worker = worker
        self.goal_reach = False
        self.current_pose = Pose2D()
        self.goal_pose = Pose2D()
        self.active = False

        self.cmd_pub = self.worker.create_publisher(Twist, self.worker.topic.cmd_vel, qos_profile=1)
    
    def current_pose2D_from_pose_stamped(self, pose_stamped):
        _,_,theta = self.worker.tf_manager.euler_from_quaternion(pose_stamped.pose.orientation.x,
                                                                 pose_stamped.pose.orientation.y,
                                                                 pose_stamped.pose.orientation.z,
                                                                 pose_stamped.pose.orientation.w)
        
        self.current_pose.x = pose_stamped.pose.position.x
        self.current_pose.y = pose_stamped.pose.position.y
        self.current_pose.theta = theta

        return pose_stamped

    def control_loop(self):
        dp = np.array([self.goal_pose.x, self.goal_pose.y])-np.array([self.current_pose.x, self.current_pose.y])
        dist = np.linalg.norm(dp)
        
        if dist > 0.2:
            v,w = self.control_policy(dp)
            self.walk(v,w)
        else:
            self.walk(0.0, 0.0)
            self.goal_reach = True
            self.do_logging('Robot is stopped.')

    def control_policy(self, err):
            v = self.k_v
            e = math.atan2(err[1],err[0])-self.current_pose.theta
            w = self.k_w*math.atan2(math.sin(e),math.cos(e)) 
            return v,w
    
    def walk(self, v, w):
        if not self.worker.working:
            self.do_logging('turtle walk while worker is not active -> pass')
            return

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)

    def do_logging(self, msg):
        self.worker.get_logger().info("[{}] {}".format(self.module_name, msg))


        
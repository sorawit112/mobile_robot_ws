from geometry_msgs.msg import Pose2D, Twist
from turtlesim.msg import Pose
import numpy as np
import math


class Controller(object):
    k_v = 3.0
    k_w = 10.0

    def __init__(self, worker):
        self.module_name = "controller"
        self.worker = worker
        self.goal_reach = False
        self.current_pose = Pose2D()
        self.goal_pose = Pose2D()
        self.active = False

        self.cmd_pub = self.worker.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile=1)
        self.worker.create_subscription(Pose,'/turtle1/pose',self.pose_callback,10)
    
    def turtle_pose_cb(self, msg):
        if self.active:
            self.current_pose.x = msg.x
            self.current_pose.y = msg.y
            self.current_pose.theta = msg.theta

    def control_loop(self):
        dp = np.array([self.goal_pose.x, self.goal_pose.y])-np.array([self.current_pose.x, self.current_pose.y])
        dist = np.linalg.norm(dp)
        msg = Twist()
        if dist > 0.2:
            v,w = self.control_policy(dp)
            msg.linear.x = v
            msg.angular.z = w
            self.cmd_pub.publish(msg)
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.goal_reach = True
            self.do_logging('Robot is stopped.')

    def control_policy(self, err):
            v = self.k_v
            e = math.atan2(err[1],err[0])-self.current_pose.theta
            w = self.k_w*math.atan2(math.sin(e),math.cos(e)) 
            return v,w

    def do_logging(self, msg):
        self.worker.get_logger().info("[{}] {}".format(self.module_name, msg))


        
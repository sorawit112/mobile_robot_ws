from geometry_msgs.msg import Pose2D, Twist
from turtlesim.msg import Pose
import numpy as np
import math


class Controller(object):
    k_v = 0.1
    k_w = 0.2

    def __init__(self, node):
        self.module_name = "controller"
        self.node = node
        self.goal_reach = False
        self.current_pose = Pose2D()
        self.goal_pose = Pose2D()
        self.active = False
        self.control_state = 0

        self.cmd_pub = self.node.create_publisher(Twist, self.node.topic.cmd_vel, qos_profile=1)
    
    def current_pose2D_from_pose_stamped(self, pose_stamped):
        _,_,theta = self.node.tf_manager.euler_from_quaternion(pose_stamped.pose.orientation.x,
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
        dangle = self.goal_pose.theta - self.current_pose.theta
        print(f'd_angle:{dangle}')

        if dist > 0.2:
            print(f'state:{self.control_state}')
            if self.control_state == 0:
                if abs(dangle) < 0.17:
                    print('finish state 0 -> 1')
                    self.control_state = 1
                    self.stop()
                else:
                    v,w = self.spin(dangle)
            
            if self.control_state == 1:
                v,w = self.control_policy(dp)
            
            self.walk(v,w)
        else:
            self.stop()
            self.goal_reach = True
            self.do_logging('Robot is stopped.')

    def control_policy(self, err):
            v = self.k_v
            e = math.atan2(err[1],err[0])-self.current_pose.theta
            w = self.k_w*math.atan2(math.sin(e),math.cos(e)) 
            return v,w
    
    def spin(self, err):
            w = 0.3*np.sign(err)
            return 0.0, w
    
    def walk(self, v, w):
        if not self.node.working:
            self.do_logging('turtle walk while node is not active -> pass')
            return

        print(f'v_x:{v}, v_z:{w}')

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)
    
    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    def do_logging(self, msg):
        self.node.get_logger().info("[{}] {}".format(self.module_name, msg))


        
import rclpy

import PyKDL
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped


class TransformManager(object):
    def __init__(self, node):
        self.module_name = "transform manager"
        self.node = node
        self.tf_br = TransformBroadcaster(self.node)
        self.static_tf_br = StaticTransformBroadcaster(self.node)

        self.tf_buffer = Buffer()
        self.tf_lis = TransformListener(self.tf_buffer, self.node)

    def publish_tf(self, tf, delay=0):
        time_stamp = self.node.get_clock().now() + rclpy.time.Duration(seconds=delay)
        tf.header.stamp = time_stamp.to_msg()
        self.tf_br.sendTransform(tf)

    def publish_odom_tf(self, robot_pose_kdl, stamp=None, delay=0):
        base_footprint_tf = self.get_tf("odom", "base_footprint", stamp)[0]
        base_footprint_kdl = self.transform_to_kdl(base_footprint_tf)
        base_footprint_kdl_inverse = base_footprint_kdl.Inverse()
        # map->robot_pose*base_footprint->odom ; robot_pose ~= base_footprint
        # map->odom
        odom_kdl = robot_pose_kdl*base_footprint_kdl_inverse
        odom_tf = self.kdl_to_transform(odom_kdl, "map", "odom")

        self.publish_tf(odom_tf, delay)

    def get_tf(self, src_frame, target_frame, stamp=None):
        try:
            if stamp is None:
                stamp = self.node.get_clock().now()
            tf = self.tf_buffer.lookup_transform(src_frame, 
                                                target_frame, 
                                                stamp.to_msg(), 
                                                timeout=rclpy.time.Duration(seconds=1.0))
            return tf, True
        except (LookupException, ConnectivityException, ExtrapolationException) as e :
            self.do_logging('Could not get transform {0} to {1}'.format(src_frame, target_frame))
            self.do_logging('{}'.format(e))
            return None, False

    @staticmethod
    def kdl_to_transform(self, kdl, parent_frame, child_frame):
        tf = TransformStamped()
        tf.header.frame_id = parent_frame
        tf.child_frame_id = child_frame
        tf.transform.translation.x = kdl.p.x()
        tf.transform.translation.y = kdl.p.y()
        tf.transform.translation.z = kdl.p.z()

        x,y,z,w = kdl.M.GetQuaternion()
        tf.transform.rotation.x = x
        tf.transform.rotation.y = y
        tf.transform.rotation.z = z
        tf.transform.rotation.w = w

        return tf

    @staticmethod
    def transform_to_kdl(self, tf):
        translation = PyKDL.Vector(tf.transform.translation.x,
                                   tf.transform.translation.y,
                                   tf.transform.translation.z)
        
        rotation = PyKDL.Rotation.Quaternion(tf.transform.rotation.x,
                                             tf.transform.rotation.y,
                                             tf.transform.rotation.z,
                                             tf.transform.rotation.w)

        frame = PyKDL.Frame(rotation, translation)

        return frame
        
    @staticmethod
    def create_tf(parent_frame, child_frame):
        tf = TransformStamped()
        tf.header.frame_id = parent_frame
        tf.child_frame_id = child_frame
        tf.transform.rotation.w = 1.0

        return tf

    @staticmethod
    def pose_stamped_from_tf_stamped(tf):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = tf.header.frame_id
        pose_stamped.pose.position.x = tf.transform.translation.x
        pose_stamped.pose.position.y = tf.transform.translation.y
        pose_stamped.pose.position.z = tf.transform.translation.z
        pose_stamped.pose.orientation.x = tf.transform.rotation.x
        pose_stamped.pose.orientation.y = tf.transform.rotation.y
        pose_stamped.pose.orientation.z = tf.transform.rotation.z
        pose_stamped.pose.orientation.w = tf.transform.rotation.w

        return pose_stamped

    @staticmethod
    def tf_stamped_from_pose_stamped(posestamp, child_frame_id):
        tf = TransformStamped()
        tf.header.frame_id = posestamp.header.frame_id
        tf.child_frame_id = child_frame_id
        tf.transform.translation.x = posestamp.pose.position.x
        tf.transform.translation.y = posestamp.pose.position.y
        tf.transform.translation.z = posestamp.pose.position.z
        tf.transform.rotation.x = posestamp.pose.orientation.x
        tf.transform.rotation.y = posestamp.pose.orientation.y
        tf.transform.rotation.z = posestamp.pose.orientation.z
        tf.transform.rotation.w = posestamp.pose.orientation.w

        return tf

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        rotation = PyKDL.Rotation.RPY(roll,pitch,yaw)
        x,y,z,w = rotation.GetQuaternion()

        return x,y,z,w

    @staticmethod  
    def euler_from_quaternion(x,y,z,w):
        rotation = PyKDL.Rotation.Quaternion(x,y,z,w)
        r,p,y = rotation.GetRPY()

        return r,p,y

    def do_logging(self, msg):
        self.node.get_logger().info('[{0}] : {1}'.format(self.module_name, msg))
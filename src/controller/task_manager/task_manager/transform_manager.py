import imp
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TransformManager(object):
    def __init__(self, node):
        self.module_name = "transform manager"
        self.node = node
        self.tf_br = TransformBroadcaster(self.node)
        self.static_tf_br = StaticTransformBroadcaster(self.node)

        self.tf_buffer = Buffer()
        self.tf_lis = TransformListener(self.tf_buffer, self.node)

    def create_tf(self, frame_id, child_frame_id):
        tf = TransformStamped()
        return tf

    def publish_tf(self, tf):
        self.tf_br.sendTransform(tf)

    def get_tf(self, target_frame, src_frame):
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, src_frame, 0)
            return tf, True
        except:
            self.do_logging('Could not transform {src_frame} to {target_frame}')
            return None, False

    def tfstamp_to_posestamp(self, tf):
        pass

    def posestamp_to_tfstamp(self, posestamp, frame_id):
        pass

    def do_logging(self, msg):
        self.node.get_logger.info('[{0}] : {1}'.format(self.module_name, msg))
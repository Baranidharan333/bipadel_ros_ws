import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

class ImuToTF(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)

    def imu_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_1'
        t.transform.translation.x = 0.069374
        t.transform.translation.y = -0.001578
        t.transform.translation.z = 0.262406
        t.transform.rotation = msg.orientation
        self.br.sendTransform(t)

    # def imu_callback(self, msg):
    #     t = TransformStamped()
    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = 'odom'
    #     t.child_frame_id = 'base_link'
    #     t.transform.translation.x = -0.06302135621902447
    #     t.transform.translation.y = -0.001654303303745368
    #     t.transform.translation.z = -0.26447733595472356
    #     t.transform.rotation = msg.orientation
    #     self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
# 0.069374 -0.001578 0.262406
# xyz="0.06302135621902447 -0.001654303303745368 0.26447733595472356"
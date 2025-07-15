import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math
import random

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_fake_imu)

    def publish_fake_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Fake orientation (roll, pitch, yaw to quaternion)
        roll = random.uniform(-0.2, 0.2)
        pitch = random.uniform(-0.2, 0.2)
        yaw = random.uniform(-0.2, 0.2)
        q = self.euler_to_quaternion(roll, pitch, yaw)
        msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Fake angular velocity and acceleration
        msg.angular_velocity.x = random.uniform(-1, 1)
        msg.angular_velocity.y = random.uniform(-1, 1)
        msg.angular_velocity.z = random.uniform(-1, 1)

        msg.linear_acceleration.x = random.uniform(-0.5, 0.5)
        msg.linear_acceleration.y = random.uniform(-0.5, 0.5)
        msg.linear_acceleration.z = random.uniform(9.5, 10.5)

        self.pub.publish(msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

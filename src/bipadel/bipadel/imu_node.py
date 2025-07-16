# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import Quaternion
# import math
# import random

# class IMUPublisher(Node):
#     def __init__(self):
#         super().__init__('imu_publisher')
#         self.pub = self.create_publisher(Imu, '/imu/data', 10)
#         self.timer = self.create_timer(0.1, self.publish_fake_imu)

#     def publish_fake_imu(self):
#         msg = Imu()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'base_link'

#         # Fake orientation (roll, pitch, yaw to quaternion)
#         roll = random.uniform(-0.2, 0.2)
#         pitch = random.uniform(-0.2, 0.2)
#         yaw = random.uniform(-0.2, 0.2)
#         q = self.euler_to_quaternion(roll, pitch, yaw)
#         msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

#         # Fake angular velocity and acceleration
#         msg.angular_velocity.x = random.uniform(-1, 1)
#         msg.angular_velocity.y = random.uniform(-1, 1)
#         msg.angular_velocity.z = random.uniform(-1, 1)

#         msg.linear_acceleration.x = random.uniform(-0.5, 0.5)
#         msg.linear_acceleration.y = random.uniform(-0.5, 0.5)
#         msg.linear_acceleration.z = random.uniform(9.5, 10.5)

#         self.pub.publish(msg)

#     def euler_to_quaternion(self, roll, pitch, yaw):
#         cr = math.cos(roll / 2)
#         sr = math.sin(roll / 2)
#         cp = math.cos(pitch / 2)
#         sp = math.sin(pitch / 2)
#         cy = math.cos(yaw / 2)
#         sy = math.sin(yaw / 2)

#         qx = sr * cp * cy - cr * sp * sy
#         qy = cr * sp * cy + sr * cp * sy
#         qz = cr * cp * sy - sr * sp * cy
#         qw = cr * cp * cy + sr * sp * sy
#         return [qx, qy, qz, qw]

# def main(args=None):
#     rclpy.init(args=args)
#     node = IMUPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()



# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import Quaternion
# from bipadel_robot_bringup.msg import Imu as CustomImu
# import tf_transformations
# import math

# class ImuBridgeNode(Node):
#     def __init__(self):
#         super().__init__('imu_bridge_node')

#         # Subscription to custom IMU
#         self.subscription = self.create_subscription(
#             CustomImu,
#             'custom_imu',
#             self.listener_callback,
#             10
#         )

#         # Publisher to sensor_msgs/Imu
#         self.publisher = self.create_publisher(Imu, 'imu/data', 10)

#     def listener_callback(self, msg):
#         imu_msg = Imu()
#         imu_msg.header.stamp = self.get_clock().now().to_msg()
#         imu_msg.header.frame_id = 'base_link'

#         # Orientation - if you have it, otherwise set to 0
#         roll, pitch, yaw = 0.0, 0.0, 0.0  # Replace with your logic if available
#         q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
#         imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

#         # Angular velocity
#         imu_msg.angular_velocity.x = msg.gyro_x
#         imu_msg.angular_velocity.y = msg.gyro_y
#         imu_msg.angular_velocity.z = msg.gyro_z

#         # Linear acceleration
#         imu_msg.linear_acceleration.x = msg.accel_x
#         imu_msg.linear_acceleration.y = msg.accel_y
#         imu_msg.linear_acceleration.z = msg.accel_z

#         self.publisher.publish(imu_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ImuBridgeNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from bipadel_robot_bringup.msg import Imu as custom
from geometry_msgs.msg import Quaternion
import math
import random

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.pub = self.create_publisher(Imu, '/imu_data', 10)
        self.timer = self.create_timer(0.1, self.publish_fake_imu)
        self.subscription = self.create_subscription(custom, '/imu', self.listener_callback, 10)

        # Initialize values
        self.accel_x = self.accel_y = self.accel_z = 0.0
        self.gyro_x = self.gyro_y = self.gyro_z = 0.0
        self.vel_x = self.vel_y = self.vel_z = 0.0
        self.prev_time = self.get_clock().now()

    def listener_callback(self, msg):
        self.accel_x = msg.accel_x
        self.accel_y = msg.accel_y
        self.accel_z = msg.accel_z

        self.gyro_x = msg.gyro_x
        self.gyro_y = msg.gyro_y
        self.gyro_z = msg.gyro_z

    def accel_to_velocity(self, accel, prev_velocity, dt):
        return prev_velocity + accel * dt

    def publish_fake_imu(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        # Integrate acceleration to velocity
        self.vel_x = self.accel_to_velocity(self.accel_x, self.vel_x, dt)
        self.vel_y = self.accel_to_velocity(self.accel_y, self.vel_y, dt)
        self.vel_z = self.accel_to_velocity(self.accel_z, self.vel_z, dt)

        msg = Imu()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'imu_1'

        # Orientation from gyro values (treated as euler for simulation)
        roll = self.gyro_x
        pitch = self.gyro_y
        yaw = self.gyro_z
        q = self.euler_to_quaternion(roll, pitch, yaw)
        msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Angular velocity
        msg.angular_velocity.x = self.gyro_x
        msg.angular_velocity.y = self.gyro_y
        msg.angular_velocity.z = self.gyro_z

        # Linear acceleration
        msg.linear_acceleration.x = self.accel_x
        msg.linear_acceleration.y = self.accel_y
        msg.linear_acceleration.z = self.accel_z

        self.pub.publish(msg)

        # Optional: log velocity
        self.get_logger().info(
            f"Velocity -> x: {self.vel_x:.2f}, y: {self.vel_y:.2f}, z: {self.vel_z:.2f}"
        )

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

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# import random
# import time

# class RandomJointPublisher(Node):
#     def __init__(self):
#         super().__init__('random_joint_publisher')
#         self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
#         timer_period = 0.1
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.joint_names = [
#             'Revolute 2', 'Revolute 4', 'Revolute 5', 'Revolute 6',
#             'Revolute 7', 'Revolute 8', 'Revolute 9', 'Revolute 10',
#             'Revolute 11', 'Revolute 12', 'Revolute 13'
#         ]

#     def timer_callback(self):
#         msg = JointState()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.name = self.joint_names
#         msg.position = [random.uniform(-1.5, 1.5) for _ in self.joint_names]
#         self.publisher_.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = RandomJointPublisher()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from bipadel_robot_bringup.msg import MotorFeedback

# class MotorJointPublisher(Node):
#     def __init__(self):
#         super().__init__('motor_joint_publisher')

#         # Publisher to joint_states
#         self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

#         # Subscriber to /motor_feedback
#         self.sub = self.create_subscription(
#             MotorFeedback,
#             '/motor_feedback',
#             self.feedback_callback,
#             10
#         )

#         # Joint names (should match your URDF exactly)
#         self.joint_names = ['Revolute 2','Revolute 4','Revolute 5','Revolute 10','Revolute 6','Revolute 11',
#                     'Revolute 7', 'Revolute 12', 'Revolute 8', 'Revolute 13','Revolute 9', 

#         ]            

#     def feedback_callback(self, msg: MotorFeedback):
#         # Collect servo angles (in degrees or ADC units?) → convert to radians
#         servo_values = [0.00,
#             100+msg.servo_6, 100+msg.servo_7, 100+msg.servo_8, 100+msg.servo_9, 100+msg.servo_10,
#             100+msg.servo_11, 100+msg.servo_12, 100+msg.servo_13, 100+msg.servo_14, 100+msg.servo_15
#         ]

#         # Assuming servo values are in degrees. If in ADC (e.g. 0–1023), adjust this conversion
#         joint_positions = [float(900+val) * 3.1416 / 180.0 for val in servo_values]

#         joint_msg = JointState()
#         joint_msg.header.stamp = self.get_clock().now().to_msg()
#         joint_msg.name = self.joint_names
#         joint_msg.position = joint_positions

#         self.joint_pub.publish(joint_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = MotorJointPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from bipadel_robot_bringup.msg import MotorFeedback, SLAM  # Add SLAM import
import math

class MotorJointPublisher(Node):
    def __init__(self):
        super().__init__('motor_joint_publisher')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.sub_motor = self.create_subscription(
            MotorFeedback,
            '/motor_feedback',
            self.feedback_callback,
            10
        )

        self.sub_slam = self.create_subscription(
            SLAM,
            '/slam',
            self.slam_callback,
            10
        )

        # Joint names (ensure these match your URDF exactly)
        self.joint_names = [
            'Revolute 2', 'Revolute 4', 'Revolute 5', 'Revolute 10', 'Revolute 6',
            'Revolute 11', 'Revolute 7', 'Revolute 12', 'Revolute 8', 'Revolute 13',
            'Revolute 9'
        ]

        # Initialize joint positions
        self.joint_positions = [0.0] * len(self.joint_names)
        self.last_motor_msg = None

    def feedback_callback(self, msg: MotorFeedback):
        # Cache the message so we can combine it with SLAM
        self.last_motor_msg = msg
        self.update_joint_state()

    def slam_callback(self, msg: SLAM):
        # Update 'Revolute 2' angle (index 0) with slam.angle in radians
        self.joint_positions[0] = math.radians(msg.angle)
        self.update_joint_state()

    def update_joint_state(self):
        if self.last_motor_msg is None:
            return

        # Convert servo values (assuming degree offset +900, then radian conversion)
        servo_values = [
            100 + self.last_motor_msg.servo_6,
            100 + self.last_motor_msg.servo_7,
            100 + self.last_motor_msg.servo_8,
            100 + self.last_motor_msg.servo_9,
            100 + self.last_motor_msg.servo_10,
            100 + self.last_motor_msg.servo_11,
            100 + self.last_motor_msg.servo_12,
            100 + self.last_motor_msg.servo_13,
            100 + self.last_motor_msg.servo_14,
            100 + self.last_motor_msg.servo_15,
        ]

        # Convert to radians and assign to joint_positions[1:] (skip Revolute 2)
        for i in range(1, len(self.joint_positions)):
            self.joint_positions[i] = math.radians(900 + servo_values[i - 1])

        # Create and publish the joint message
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = self.joint_positions

        self.joint_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

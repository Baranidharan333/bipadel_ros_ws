import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import random
import time

class RandomJointPublisher(Node):
    def __init__(self):
        super().__init__('random_joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joint_names = [
            'Revolute 2', 'Revolute 4', 'Revolute 5', 'Revolute 6',
            'Revolute 7', 'Revolute 8', 'Revolute 9', 'Revolute 10',
            'Revolute 11', 'Revolute 12', 'Revolute 13'
        ]

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [random.uniform(-1.5, 1.5) for _ in self.joint_names]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RandomJointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

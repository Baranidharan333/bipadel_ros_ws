import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time

class FakeLaserScanner(Node):
    def __init__(self):
        super().__init__('fake_laserscanner')
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)
        self.time_offset = time.time()
        self.get_logger().info("Fake LaserScanner Node Started")

    def publish_scan(self):
        now = time.time() - self.time_offset
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2 * math.pi
        scan_msg.angle_increment = math.radians(1)
        scan_msg.range_min = 0.02
        scan_msg.range_max = 4.0

        num_readings = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
        ranges = [float('inf')] * num_readings

        # Simulate Ultrasonic 1 (0–180°)
        for i in range(180):
            angle = math.radians(i)
            # Simulated dynamic obstacle movement
            ranges[i] = 1.0 + 0.3 * math.sin(angle + now)

        # Simulate Ultrasonic 2 (180–360°)
        for i in range(180, 360):
            angle = math.radians(i)
            # Simulated dynamic obstacle movement
            ranges[i] = 1.5 + 0.3 * math.cos(angle + now * 1.2)

        scan_msg.ranges = ranges
        scan_msg.intensities = [0.0] * num_readings
        self.publisher.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserScanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# import math
# import time

# class LaserScanner(Node):
#     def __init__(self):
#         super().__init__('laserscanner')
#         self.publisher = self.create_publisher(LaserScan, '/scan', 10)
#         self.timer = self.create_timer(0.1, self.publish_scan)
#         self.time_offset = time.time()
#         self.get_logger().info("LaserScanner Node Started")

#     def publish_scan(self):
#         now = time.time() - self.time_offset
#         scan_msg = LaserScan()
#         scan_msg.header.stamp = self.get_clock().now().to_msg()
#         scan_msg.header.frame_id = 'lidar_1'
#         scan_msg.angle_min = 0.0
#         scan_msg.angle_max = 2 * math.pi
#         scan_msg.angle_increment = math.radians(1)
#         scan_msg.range_min = 0.02
#         scan_msg.range_max = 4.0

#         num_readings = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
#         ranges = [float('inf')] * num_readings

#         # Simulate Ultrasonic 1 (0–180°)
#         for i in range(180):
#             angle = math.radians(i)
#             # Simulated dynamic obstacle movement
#             ranges[i] = 1.0 + 0.3 * math.sin(angle + now)

#         # Simulate Ultrasonic 2 (180–360°)
#         for i in range(180, 360):
#             angle = math.radians(i)
#             # Simulated dynamic obstacle movement
#             ranges[i] = 1.5 + 0.3 * math.cos(angle + now * 1.2)

#         scan_msg.ranges = ranges
#         scan_msg.intensities = [0.0] * num_readings
#         self.publisher.publish(scan_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = LaserScanner()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from bipadel_robot_bringup.msg import SLAM  # Custom message with s1, s2, angle
# import math

# class SLAMNode(Node):
#     def __init__(self):
#         super().__init__('slam_node')
#         self.publisher = self.create_publisher(LaserScan, 'scan', 10)
#         self.subscription = self.create_subscription(SLAM, 'slam', self.listener_callback, 10)

#         # Initialize default range array
#         self.num_readings = 360
#         self.ranges = [float('inf')] * self.num_readings
#         self.get_logger().info("SLAM LaserScan Node Started")

#         self.timer = self.create_timer(0.01, self.publish_scan)

#     def listener_callback(self, msg):
#         """Update ranges with two ultrasonic distance readings"""
#         # Clamp angles within 0–359
#         index1 = max(0, min(359, int(msg.angle)))
#         index2 = max(0, min(359, int(180 + msg.angle)))  # s2 is on the opposite side
#         print(msg.s1)
#         self.ranges[index1] = msg.s1
#         self.ranges[index2] = msg.s2

#         self.get_logger().info(f"Updated ranges[{index1}] = {msg.s1}, ranges[{index2}] = {msg.s2}")

#     def publish_scan(self):
#         scan_msg = LaserScan()
#         scan_msg.header.stamp = self.get_clock().now().to_msg()
#         scan_msg.header.frame_id = 'lidar_1'

#         scan_msg.angle_min = 0.0
#         scan_msg.angle_max = 2 * math.pi
#         scan_msg.angle_increment = math.radians(1)
#         scan_msg.range_min = 0.02
#         scan_msg.range_max = 4.0

#         scan_msg.ranges = self.ranges
#         scan_msg.intensities = [0.0] * self.num_readings
#         self.publisher.publish(scan_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = SLAMNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from bipadel_robot_bringup.msg import SLAM
import math
import time


class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.subscription = self.create_subscription(SLAM, '/slam', self.listener_callback, 10)

        self.num_readings = 360
        self.ranges = [float('inf')] * self.num_readings
        self.last_update_time = [0.0] * self.num_readings

        self.timer = self.create_timer(0.1, self.publish_scan)

        self.get_logger().info("✅ SLAM LaserScan Node Started")

    def listener_callback(self, msg):
        now = time.time()

        angle_deg = int(msg.angle)
        index1 = max(0, min(359, angle_deg))
        index2 = (index1 + 180) % 360

        s1_m = msg.s1 / 100.0  # Convert cm to meters
        s2_m = msg.s2 / 100.0

        if 0.02 <= s1_m <= 4.0:
            self.ranges[index1] = s1_m
            self.last_update_time[index1] = now

        if 0.02 <= s2_m <= 4.0:
            self.ranges[index2] = s2_m
            self.last_update_time[index2] = now

        self.get_logger().info(
            f"Updated: angle={angle_deg}, s1={s1_m:.2f}m, s2={s2_m:.2f}m → indices: {index1}, {index2}"
        )

    def publish_scan(self):
        now = time.time()
        timeout = 10  # seconds

        # Reset old data to .inf
        for i in range(self.num_readings):
            if now - self.last_update_time[i] > timeout:
                self.ranges[i] = float('inf')

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_1'

        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = math.radians(1)
        scan.range_min = 0.02
        scan.range_max = 4.0

        scan.ranges = self.ranges
        scan.intensities = [0.0] * self.num_readings

        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


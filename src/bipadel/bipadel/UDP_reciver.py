import rclpy
from rclpy.node import Node
from bipadel_robot_bringup.msg import Imu, MotorFeedback, SLAM
import socket
import re

class UDPReceiverNode(Node):
    def __init__(self):
        super().__init__('udp_receiver_node')

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.slam_pub = self.create_publisher(SLAM, 'slam', 10)
        self.motor_pub = self.create_publisher(MotorFeedback, 'motor_feedback', 10)

        # UDP Setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 8888))
        self.get_logger().info("Listening on UDP port 8888")

        self.timer = self.create_timer(0.0001, self.receive_data)

    def receive_data(self):
        try:
            self.sock.settimeout(0.001)
            data, _ = self.sock.recvfrom(256)
            message = data.decode().strip()
            self.parse_data(message)
            # print(message)
        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().error(f"UDP receive error: {e}")

    def parse_data(self, data):
        if "Accel:" in data and "Gyro:" in data:
            match = re.search(r"Accel:\(([^)]+)\),Gyro:\(([^)]+)\)", data)
            if match:
                accel = tuple(map(float, match.group(1).split(',')))
                gyro = tuple(map(float, match.group(2).split(',')))

                msg = Imu()
                msg.accel_x, msg.accel_y, msg.accel_z = accel
                msg.gyro_x, msg.gyro_y, msg.gyro_z = gyro
                self.imu_pub.publish(msg)
                # print(msg)

        elif "S_1:" in data and "S_2:" in data and "C_A:" in data:
            match = re.search(r"S_1:\s*([\d.]+)\s*\|\s*S_2:\s*([\d.]+)\s*\|\s*C_A:\s*(\d+)", data)
            if match:
                msg = SLAM()
                # print(match.group(1))
                msg.s1 = float(match.group(1))
                msg.s2 = float(match.group(2))
                msg.angle = int(match.group(3))
                self.slam_pub.publish(msg)
                # print(msg)

        elif any(key in data for key in ["Right Body Front", "Home", "Center Tilt", "Right Tilt", 
                                          "Left Leg Bend", "Right Leg Straight", "Center Tilt Again", 
                                          "Left Tilt", "Right Leg Bend", "Left Leg Straight", "Left Body Front"]):

            commands = dict(re.findall(r'c;(\d+):(\d+)', data))
            pots = dict(re.findall(r'P;(\d+):\s*(\d+)', data))

            msg = MotorFeedback()
            for i in range(6, 16):
                setattr(msg, f'servo_{i}', int(commands.get(str(i), 0)))

            for pin in [33, 32, 35, 34, 39, 27, 14, 25, 26, 36]:
                setattr(msg, f'pot_{pin}', int(pots.get(str(pin), 0)))

            self.motor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UDPReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
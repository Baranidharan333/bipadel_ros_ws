from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame']
        ),
        Node(
            package='custom_ultrasonic_laserscanner',
            executable='fake_laserscanner_node',
            name='fake_laserscanner',
            output='screen'
        ),
    ])

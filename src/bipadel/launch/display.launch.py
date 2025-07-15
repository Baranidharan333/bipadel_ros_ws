from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'bipadel'
    urdf_file_name = 'robot_urd.xacro'
    rviz_config_file = 'rviz_config.rviz'

    urdf_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'urdf',
        urdf_file_name
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        rviz_config_file
    ])

    return LaunchDescription([
        # robot_state_publisher (publishes TFs using joint states and URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path])
            }]
        ),

        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),

        # Random Joint Publisher node from your bipadel package
        Node(
            package=pkg_name,
            executable='random_joint_publisher',
            name='random_joint_publisher',
            output='screen'
        ),
        Node(
            package="bipadel",
            executable='imu_node',
            name='imu_node',
            output='screen',
        ),
        Node(
            package='bipadel',
            executable='SLAM',
            name='SLAM',
            output='screen'
        ),
        Node(
            package='bipadel',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        Node(
            package='bipadel',
            executable='imu_broadcaster',
            name='imu_broadcaster',
            output='screen'
        ),
    ])

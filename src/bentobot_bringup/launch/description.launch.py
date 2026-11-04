from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_transform',
            arguments=['0.043', '0', '0.168', '0', '0', '-0.7071', '0.7071', 'base_link', 'base_laser'],
        ),
    ])

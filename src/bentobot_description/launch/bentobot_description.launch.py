import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('bentobot_description'), 'urdf', 'bentobot.xacro')
    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # Send fake joint values
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),

        # Send robot states to tf
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
    ])

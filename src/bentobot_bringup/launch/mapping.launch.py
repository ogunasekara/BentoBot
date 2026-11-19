"""Mapping mode: slam_toolbox (async) building /map from /scan and TF.

Started and stopped by bentobot_web, or included by teleop.launch.py.
Needs base.launch.py (or equivalent drivers) to be running.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('bentobot_bringup')
    default_config = os.path.join(bringup_dir, 'config', 'slam_toolbox_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('slam_config', default_value=default_config),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[LaunchConfiguration('slam_config')],
        ),
    ])

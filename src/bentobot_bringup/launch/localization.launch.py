"""Localization mode: nav2 map_server + AMCL on a saved (possibly edited) map.

  ros2 launch bentobot_bringup localization.launch.py map:=/bentobot_ws/data/maps/<id>/map.yaml

AMCL publishes the map -> odom TF and /amcl_pose; set the initial pose on
/initialpose. Started and stopped by bentobot_web. Needs base.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('bentobot_bringup')
    amcl_config = os.path.join(bringup_dir, 'config', 'amcl_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', description='path to the map YAML'),
        DeclareLaunchArgument('amcl_config', default_value=amcl_config),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('map')}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[LaunchConfiguration('amcl_config')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),
    ])

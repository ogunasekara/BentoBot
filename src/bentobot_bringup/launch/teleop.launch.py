import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource, PythonLaunchDescriptionSource)
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('bentobot_bringup')
    rosbridge_dir = get_package_share_directory('rosbridge_server')

    def bringup_launch(name):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', name)))

    robot_config = os.path.join(bringup_dir, 'config', 'bentobot_config.yaml')
    slam_config = os.path.join(bringup_dir, 'config', 'slam_toolbox_config.yaml')

    return LaunchDescription([
        # Foxglove Studio Integration (connect with "Rosbridge", ws://<robot>:9090)
        IncludeLaunchDescription(AnyLaunchDescriptionSource(
            os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml'))),

        # Robot Configuration
        bringup_launch('description.launch.py'),

        # Teleoperation
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy',
            output='screen',
            parameters=[{'dev': '/dev/input/js0'}],
        ),
        Node(
            package='bentobot_teleop',
            executable='bentobot_teleop_node',
            name='bentobot_teleop',
        ),

        # Sensors and Actuators
        Node(
            package='bentobot_mcu_bridge',
            executable='bentobot_mcu_bridge_node',
            name='bentobot_mcu_bridge',
        ),
        bringup_launch('lidar.launch.py'),

        # Odometry
        Node(
            package='bentobot_odom',
            executable='odom_node',
            name='bentobot_odom',
            parameters=[robot_config],
        ),

        # Mapping
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config],
        ),
    ])

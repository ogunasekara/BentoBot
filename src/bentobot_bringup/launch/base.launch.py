"""Always-on part of the robot: drivers, teleop, odometry and the web UI.

Mapping and localization are separate launch files (mapping.launch.py,
localization.launch.py) that bentobot_web starts and stops as subprocesses.

Arguments:
  startup_mode  idle | mapping   mode bentobot_web starts once it is up
  web           true | false     run the bentobot_web control panel
  web_port      HTTP port (default $BENTOBOT_WEB_PORT or 8080)
  web_iface     interface to bind, e.g. wlan0, or an IP such as 0.0.0.0
                (default $BENTOBOT_WEB_IFACE or wlan0)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource, PythonLaunchDescriptionSource)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('bentobot_bringup')
    rosbridge_dir = get_package_share_directory('rosbridge_server')

    def bringup_launch(name):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', name)))

    robot_config = os.path.join(bringup_dir, 'config', 'bentobot_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('startup_mode', default_value='idle'),
        DeclareLaunchArgument('web', default_value='true'),
        DeclareLaunchArgument(
            'web_port', default_value=EnvironmentVariable('BENTOBOT_WEB_PORT', default_value='8080')),
        DeclareLaunchArgument(
            'web_iface', default_value=EnvironmentVariable('BENTOBOT_WEB_IFACE', default_value='wlan0')),

        # Foxglove Studio Integration (connect with "Rosbridge", ws://<robot>:9090)
        IncludeLaunchDescription(AnyLaunchDescriptionSource(
            os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml'))),

        # Robot Configuration
        bringup_launch('description.launch.py'),

        # Teleoperation. autorepeat keeps /joy flowing while the sticks are
        # still, so the teleop watchdog and the web UI can tell when the
        # controller is gone. respawn lets bentobot_web restart joy by killing it.
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy',
            output='screen',
            parameters=[{'dev': '/dev/input/js0', 'autorepeat_rate': 20.0}],
            respawn=True,
            respawn_delay=1.0,
        ),
        Node(
            package='bentobot_teleop',
            executable='bentobot_teleop_node',
            name='bentobot_teleop',
            parameters=[robot_config],
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

        # Web control panel (http://<robot>:8080)
        Node(
            package='bentobot_web',
            executable='bentobot_web_node',
            name='bentobot_web',
            output='screen',
            condition=IfCondition(LaunchConfiguration('web')),
            parameters=[{
                'startup_mode': LaunchConfiguration('startup_mode'),
                'port': LaunchConfiguration('web_port'),
                'iface': LaunchConfiguration('web_iface'),
            }],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_dir = get_package_share_directory('bentobot_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # Robot pose
    pose_args = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']

    return LaunchDescription(
        [DeclareLaunchArgument(arg, default_value='0') for arg in pose_args] + [
            DeclareLaunchArgument('world', default_value='worlds/willowgarage.world'),

            # Convert robot description from xacro to urdf
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(description_dir, 'launch', 'bentobot_description.launch.py'))),

            # Launch Gazebo World
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
                launch_arguments={'world': LaunchConfiguration('world')}.items(),
            ),

            # Spawn My Robot
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                output='screen',
                arguments=[
                    '-topic', 'robot_description', '-entity', 'bentobot',
                    '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                    '-z', LaunchConfiguration('z'), '-R', LaunchConfiguration('roll'),
                    '-P', LaunchConfiguration('pitch'), '-Y', LaunchConfiguration('yaw'),
                ],
            ),
        ])

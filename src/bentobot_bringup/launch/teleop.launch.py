"""Base bringup + mapping, without going through the web UI's mode manager.

The web UI still runs and shows this as an external mapping run that can be
saved but not stopped from the browser. `docker compose up` runs
base.launch.py startup_mode:=mapping instead, which the UI fully controls.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('bentobot_bringup')

    def bringup_launch(name, **args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', name)),
            launch_arguments=args.items())

    return LaunchDescription([
        bringup_launch('base.launch.py', startup_mode='idle'),
        bringup_launch('mapping.launch.py'),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('topic_name', default_value='scan'),
        DeclareLaunchArgument('frame_id', default_value='base_laser'),
        DeclareLaunchArgument('port_name', default_value='/dev/ttyUSB0'),

        # ldlidar message publisher node
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='LD06',
            output='screen',
            parameters=[{
                'product_name': 'LDLiDAR_LD06',
                'topic_name': 'scan_raw',
                'frame_id': LaunchConfiguration('frame_id'),
                'port_name': LaunchConfiguration('port_name'),
                'port_baudrate': 230400,
                # true = counterclockwise, false = clockwise
                'laser_scan_dir': True,
                # zero out the distance and intensity data where
                # angle_crop_min <= angle <= angle_crop_max (degrees)
                'enable_angle_crop_func': False,
                'angle_crop_min': 135.0,
                'angle_crop_max': 225.0,
            }],
        ),

        # the driver's beam count varies scan to scan, which slam_toolbox rejects
        Node(
            package='bentobot_laser',
            executable='scan_resampler_node',
            name='scan_resampler',
            remappings=[('scan', LaunchConfiguration('topic_name'))],
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    rplidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_a1_launch.py',
    )
    rplidar_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch),
    )

    ARG_PARAM_lidar_resolution = DeclareLaunchArgument(
        'lidar_resolution',
        default_value=60,
    )
    ARG_PARAM_num_particles = DeclareLaunchArgument(
        'num_particles',
        default_value=100,
    )

    return LaunchDescription([
        ARG_PARAM_lidar_resolution,
        ARG_PARAM_num_particles,
        Node(
            package="pf_localization",
            executable="pf_node.py",
            name="pf_localization",
            parameters=[{
                "debug": True,
                'num_particles': LaunchConfiguration('num_particles'),
                'lidar_resolution': LaunchConfiguration('lidar_resolution')}]
        ),

        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[{
                "port": 8765,
                "address": "0.0.0.0",
                "tls": False,
                "topic_whitelist": [".*"],
                "max_qos_depth": 10,
            }],
        ),
        rplidar_launch_action,
    ])
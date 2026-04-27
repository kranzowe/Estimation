from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Expect someone else to run the lidar

    ARG_PARAM_lidar_resolution = DeclareLaunchArgument(
        'lidar_resolution',
        default_value='60',
    )
    ARG_PARAM_num_particles = DeclareLaunchArgument(
        'num_particles',
        default_value='100',
    )
    ARG_PARAM_x0_pos = DeclareLaunchArgument(
        'x0_pos',
        default_value='[-28.0, 7.0]',
    )
    ARG_PARAM_x0_spread = DeclareLaunchArgument(
        'x0_spread',
        default_value='1.0',
    )

    return LaunchDescription([
        ARG_PARAM_lidar_resolution,
        ARG_PARAM_num_particles,
        ARG_PARAM_x0_pos,
        ARG_PARAM_x0_spread,
        Node(
            package="pf_localization",
            executable="pf_node.py",
            name="pf_localization",
            parameters=[{
                "visualize": True,
                'num_particles': LaunchConfiguration('num_particles'),
                'lidar_resolution': LaunchConfiguration('lidar_resolution'),
                'x0_pos': LaunchConfiguration('x0_pos'),
                'x0_spread': LaunchConfiguration('x0_spread'),
            }]
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
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    return LaunchDescription([
                Node(
            package="pf_localization",
            executable="pf_node.py",
            name="pf_localization",
            parameters=[{"debug": False}]
        ),
        rplidar_launch_action,
    ])
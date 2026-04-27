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

    ARG_PARAM_x0_pos = DeclareLaunchArgument(
        'x0_pos',
        default_value='[-28.0, 7.0]',
    )
    ARG_PARAM_x0_spread = DeclareLaunchArgument(
        'x0_spread',
        default_value='1.0',
    )

    return LaunchDescription([
        ARG_PARAM_x0_pos,
        ARG_PARAM_x0_spread,
        Node(
            package="pf_localization",
            executable="pf_node.py",
            name="pf_localization",
            parameters=[{
                "visualize": False,
                'x0_pos': LaunchConfiguration('x0_pos'),
                'x0_spread': LaunchConfiguration('x0_spread'),
            }]
        ),
        rplidar_launch_action,
    ])
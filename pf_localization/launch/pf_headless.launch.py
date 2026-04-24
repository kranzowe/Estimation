from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
                Node(
            package="pf_localization",
            executable="pf_node.py",
            name="pf_localization",
            parameters=[{"debug": False}]
        ),
    ])
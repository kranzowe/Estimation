from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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
                "debug": True,
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
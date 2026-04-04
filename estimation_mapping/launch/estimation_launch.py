#!/usr/bin/env python3
"""
ROS2 Launch file for Estimation subsystem
Launches SLAM Toolbox and EKF for mapping and state estimation
"""

import os
from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('estimation_mapping')
    slam_config = os.path.join(package_share_dir, 'config', 'mapper_params.yaml')
    # ekf_config = os.path.join(package_share_dir, 'config', 'ekf.yaml')
    urdf_file = os.path.join(package_share_dir, 'urdf', 'simple.urdf')

    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content
        }],
    )

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
    )

    # Delay SLAM startup to allow rover node to initialize first
    delayed_slam = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='Starting slam_toolbox after 5 second delay'),
            slam_toolbox_node,
        ]
    )

    # EKF node for sensor fusion
    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_config],
    # )

    log_info = LogInfo(
        msg=[
            'Launching Estimation subsystem:\n',
            '  SLAM config: ', slam_config, '\n',
            # '  EKF config: ', ekf_config, '\n',
        ]
    )

    return LaunchDescription([
        log_info,
        delayed_slam,
        # ekf_node,
        robot_state_publisher_node,
    ])

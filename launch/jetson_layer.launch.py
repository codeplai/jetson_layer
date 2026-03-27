#!/usr/bin/env python3
"""MINEBOT-Q Jetson Orin NX — Master Launch File."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('jetson_layer')
    config_file = os.path.join(pkg_dir, 'config', 'jetson_layer.yaml')

    sim_arg = DeclareLaunchArgument(
        'sim', default_value='false',
        description='Simulation mode (true=no hardware)')

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='INFO',
        description='Log level for all nodes')

    sim = LaunchConfiguration('sim')
    log_level = LaunchConfiguration('log_level')

    camera_node = Node(
        package='jetson_layer',
        executable='camera_node',
        name='camera_node',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', log_level],
        condition=UnlessCondition(sim),
    )

    lidar_node = Node(
        package='jetson_layer',
        executable='lidar_node',
        name='lidar_node',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', log_level],
        condition=UnlessCondition(sim),
    )

    go2_bridge_node = Node(
        package='jetson_layer',
        executable='go2_bridge_node',
        name='go2_bridge_node',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', log_level],
    )

    state_publisher_node = Node(
        package='jetson_layer',
        executable='state_publisher_node',
        name='state_publisher_node',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', log_level],
    )

    rosbridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'rosbridge_launch.py')
        ),
    )

    return LaunchDescription([
        sim_arg,
        log_level_arg,
        camera_node,
        lidar_node,
        go2_bridge_node,
        state_publisher_node,
        rosbridge,
    ])

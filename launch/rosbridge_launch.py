#!/usr/bin/env python3
"""MINEBOT-Q Rosbridge WebSocket Launch — exposes ROS2 topics via WS :9090."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'use_compression': True,
            'max_message_size': 10000000,
            'unregister_timeout': 10.0,
        }],
        output='screen',
    )

    return LaunchDescription([
        rosbridge_node,
    ])

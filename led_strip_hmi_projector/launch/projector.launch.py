#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('led_strip_hmi_visualization'),
        'config',
        'strips.yaml',
    )
    return LaunchDescription(
        [
            Node(
                package='led_strip_hmi_projector',
                executable='projector_node',
                name='led_strip_hmi_projector',
                output='screen',
                parameters=[{'strips_config': cfg}],
            ),
        ]
    )

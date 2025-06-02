#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('led_strip_hmi_visualization')
    strips_yaml = os.path.join(pkg_share, 'config', 'strips.yaml')

    return LaunchDescription([
        Node(
            package='led_strip_hmi_visualization',
            executable='strip_visualizer',
            name='strip_visualizer',
            output='screen',
            parameters=[{'strips_config': strips_yaml}],
        ),
    ])

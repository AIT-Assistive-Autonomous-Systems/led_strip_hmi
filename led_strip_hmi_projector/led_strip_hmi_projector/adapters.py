#!/usr/bin/env python3
"""
Adapter functions for converting ProjectorConfig.

This module provides utilities to convert configuration objects into ROS2 messages
for LED strip visualization.

"""

from geometry_msgs.msg import Point
from led_strip_hmi_common.config import ProjectorConfig
from rclpy.time import Time
from led_strip_hmi_common.virtual_strip import VirtualStrip

from led_strip_hmi_msgs.msg import LEDStripPhysicalSegment, LEDStripPhysicalConfig, LEDStripPhysicalConfigArray

def get_physical_strip_config_array_msg(virtual_strip: VirtualStrip, time: Time) -> LEDStripPhysicalConfigArray:
    """
    Convert physical strip configuration to ROS message format.

    Returns
    -------
        LEDStripPhysicalConfigArray message with all physical strips.

    """
    msg = LEDStripPhysicalConfigArray()
    msg.header.stamp = time.to_msg()
    msg.header.frame_id = virtual_strip.frame_id

    for phys_id, segments in virtual_strip.physical_strips.items():
        config = LEDStripPhysicalConfig()
        config.physical_segment_id = int(phys_id)
        config.segments = []
        for seg in segments:
            seg_msg = LEDStripPhysicalSegment()
            seg_msg.num_leds = int(seg['num_leds'])
            seg_msg.start_ratio = float(seg['start'])
            seg_msg.stop_ratio = float(seg['stop'])
            config.segments.append(seg_msg)
        msg.configs.append(config)

    return msg


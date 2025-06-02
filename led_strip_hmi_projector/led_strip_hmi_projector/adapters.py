#!/usr/bin/env python3
"""
Adapter functions for converting ProjectorConfig (including VirtualStrip and YAML config)
into ROS2 LED strip configuration messages.
"""

from rclpy.time import Time

from geometry_msgs.msg import Point
from led_strip_hmi_msgs.msg import (
    LEDStrip,
    LEDStripConfig,
    VirtualLEDStripSegment,
    VirtualPerceptionConfig,
)
from led_strip_hmi_common.config import ProjectorConfig


def build_led_strip_config(
    cfg: ProjectorConfig,
    now: Time,
) -> LEDStripConfig:
    """
    Create a latched LEDStripConfig message from ProjectorConfig.

    This function populates the header, real-strip polygons, virtual-segment
    definitions, and virtual-perception parameters into a single LEDStripConfig
    message ready for publishing.

    Args:
        cfg (ProjectorConfig): Configuration object containing both the raw
            YAML parameters and the instantiated VirtualStrip.
        now (Time): Current ROS2 time used for the message header stamp.

    Returns:
        LEDStripConfig: Fully populated configuration message.
    """
    cfg_msg = LEDStripConfig()
    # Header
    cfg_msg.header.stamp = now.to_msg()
    cfg_msg.header.frame_id = cfg.strip_frame

    # Closed loop and strip order
    cfg_msg.closed = cfg.virtual_strip.closed
    cfg_msg.order = cfg.virtual_strip.order

    # Virtual segments
    for virt_segment in cfg.virtual_strip.segments:
        seg_msg = VirtualLEDStripSegment()
        seg_msg.base_strip_name = virt_segment.strip
        seg_msg.start = float(virt_segment.start)
        seg_msg.length = float(virt_segment.length)
        seg_msg.gap = bool(virt_segment.gap)
        seg_msg.pts3d = [Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2]))
                        for pt in virt_segment.pts3d]
        cfg_msg.virtual_segments.append(seg_msg)

    # Real strips
    for name, s in cfg.strips.items():
        strip_msg = LEDStrip()
        strip_msg.name = name
        for pt in s['polygon']:
            p = Point()
            p.x, p.y, p.z = float(pt[0]), float(pt[1]), float(pt[2])
            strip_msg.polygon.append(p)
        strip_msg.num_leds = int(s['num_leds'])
        cfg_msg.strips.append(strip_msg)

    # Virtual perception config
    vp_msg = VirtualPerceptionConfig()
    vp_msg.frame_id = cfg.vp_frame
    vp_msg.min_distance = cfg.min_d
    vp_msg.max_distance = cfg.max_d
    vp_msg.publish_transform = cfg.publish_tf
    vp_msg.use_led_centroid_as_frame = cfg.use_centroid
    trans = cfg.vp_offset_t
    vp_msg.offset_translation.x = float(trans[0])
    vp_msg.offset_translation.y = float(trans[1])
    vp_msg.offset_translation.z = float(trans[2])
    rot = cfg.vp_offset_r
    vp_msg.offset_rotation.x = float(rot[0])
    vp_msg.offset_rotation.y = float(rot[1])
    vp_msg.offset_rotation.z = float(rot[2])
    vp_msg.offset_rotation.w = float(rot[3])
    cfg_msg.virtual_perception = vp_msg

    return cfg_msg




#!/usr/bin/env python3
"""
ROS 2 node that visualizes LED strips in RViz by publishing static outlines,
grey LEDs, and dynamic highlight markers based on incoming projection info.
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import MarkerArray

from led_strip_hmi_msgs.msg import LEDStripProjectionInfoArray
from led_strip_hmi_common.config import ProjectorConfig

from .marker_builders import build_static_markers, build_dynamic_markers


class StripVisualizer(Node):
    """
    Node that publishes static and dynamic MarkerArrays to visualize
    physical LED strip outlines and highlight projections.
    """

    def __init__(self):
        super().__init__('strip_visualizer')

        # 1) Declare and load YAML-based configuration
        default_cfg = os.path.join(
            get_package_share_directory('led_strip_hmi_visualization'),
            'config', 'strips.yaml'
        )
        self.declare_parameter('strips_config', default_cfg)
        cfg_path = self.get_parameter('strips_config').value
        self.cfg = ProjectorConfig.load_from_yaml(cfg_path)

        # 2) Static markers publisher (latched, reliable, transient local)
        static_qos = QoSProfile(depth=1)
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        self.static_pub = self.create_publisher(
            MarkerArray,
            'led_strips',
            qos_profile=static_qos,
        )

        # 3) Dynamic highlight publishers
        self.dynamic_pub = self.create_publisher(
            MarkerArray,
            'led_strips_highlight',
            qos_profile=10,
        )
        self.discrete_pub = self.create_publisher(
            MarkerArray,
            'led_strips_discrete',
            qos_profile=10,
        )

        self.leds_pub = self.create_publisher(
            MarkerArray,
            'led_strips_led',
            qos_profile=10,
        )

        # 4) Subscription to projection info
        self.create_subscription(
            LEDStripProjectionInfoArray,
            'led_indices',
            self.cb_indices,
            qos_profile=10,
        )

        # 5) Timer to re-publish static geometry at 1 Hz
        self.create_timer(1.0, self.publish_static_markers)

    def publish_static_markers(self) -> None:
        """
        Timer callback: build and publish static strip outlines and grey LEDs.
        """
        stamp = self.get_clock().now().to_msg()
        m_arr = build_static_markers(self.cfg, stamp)
        self.static_pub.publish(m_arr)

    def cb_indices(self, msg: LEDStripProjectionInfoArray) -> None:
        """
        Subscription callback: build and publish dynamic highlight markers
        based on incoming LEDStripProjectionInfoArray.
        """
        stamp = self.get_clock().now().to_msg()
        highlight_arr, discrete_arr, leds_arr = build_dynamic_markers(
            self.cfg, msg, stamp)
        self.dynamic_pub.publish(highlight_arr)
        self.discrete_pub.publish(discrete_arr)
        self.leds_pub.publish(leds_arr)


def main(args=None):
    """
    Entry point for the strip_visualizer node.
    """
    rclpy.init(args=args)
    node = StripVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

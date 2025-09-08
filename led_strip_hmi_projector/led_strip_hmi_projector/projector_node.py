#!/usr/bin/env python3
"""
ROS2 node that projects 3D detections onto a virtual LED strip.

Loads physical strip layout from YAML; builds a VirtualStrip for normalized indexing;
draws a debug image overlaying strips, detections, and projections; publishes
LED index messages and the debug image.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory

import cv2

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TransformStamped

from led_strip_hmi_common.config import ProjectorConfig
from led_strip_hmi_common.tf_utils import transform_point

from led_strip_hmi_msgs.msg import (
    LEDStripPhysicalConfigArray,
    LEDStripProjectionInfo,
    LEDStripProjectionInfoArray
)

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from vision_msgs.msg import Detection3DArray

from .adapters import get_physical_strip_config_array_msg
from .drawing import (
    create_canvas,
    draw_axes,
    draw_detections,
    draw_real_strips,
    draw_virtual_strip,
    world_to_pixel,
)
from .projector import map_detection


class ProjectorNode(Node):
    """
    Node for projecting 3D object detections onto a virtual LED strip.

    This node:
      - Loads physical strip configuration from a YAML file.
      - Publishes a latched LEDStripConfig message describing the strips.
      - Subscribes to Detection3DArray messages (and optionally LaserScan).
      - Projects detections into strip-normalized LED indices.
      - Publishes LEDStripProjectionInfoArray messages.
      - Generates and publishes a debug image showing projections.
    """

    def __init__(self):
        """
        Initialize the ProjectorNode.

        Reads parameters, loads configuration via ProjectorConfig, publishes
        the LEDStripConfig once (latched), sets up TF buffers and an optional
        static transform, creates publishers/subscribers, and logs the full
        configuration.
        """
        super().__init__('led_strip_hmi_projector')

        # load YAML
        default_cfg = os.path.join(
            get_package_share_directory('led_strip_hmi_visualization'),
            'config',
            'strips.yaml',
        )
        self.declare_parameter('strips_config', default_cfg)
        self.cfg_path: str = self.get_parameter('strips_config').value

        self.declare_parameter('debug_image', True)
        self.debug_image: bool = self.get_parameter('debug_image').value

        self.cfg = ProjectorConfig.load_from_yaml(self.cfg_path)

        # Latched config publisher
        self.config_pub = self.create_publisher(
            LEDStripPhysicalConfigArray,
            'led_strip_config',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        # --- Publish configuration once ---
        cfg_msg = get_physical_strip_config_array_msg(
            self.cfg.virtual_strip, self.get_clock().now())
        self.config_pub.publish(cfg_msg)

        # TF buffer & listener
        self.tfbuf = Buffer()
        self.tfl = TransformListener(self.tfbuf, self)

        # optionally publish strip_frame → vp_frame as a static TF
        if self.cfg.publish_tf:
            if self.cfg.use_centroid:
                cx, cy, cz = self.cfg.virtual_strip.compute_led_centroid()
            else:
                cx, cy, cz = 0.0, 0.0, 0.0

            t = TransformStamped()
            t.header.frame_id = self.cfg.strip_frame
            t.child_frame_id = self.cfg.vp_frame
            t.transform.translation.x = float(cx) + self.cfg.vp_offset_t[0]
            t.transform.translation.y = float(cy) + self.cfg.vp_offset_t[1]
            t.transform.translation.z = float(cz) + self.cfg.vp_offset_t[2]
            # identity rotation
            t.transform.rotation.x = self.cfg.vp_offset_r[0]
            t.transform.rotation.y = self.cfg.vp_offset_r[1]
            t.transform.rotation.z = self.cfg.vp_offset_r[2]
            t.transform.rotation.w = self.cfg.vp_offset_r[3]

            self._static_broadcaster = StaticTransformBroadcaster(self)
            self._static_broadcaster.sendTransform(t)

        # cv bridge, publishers & subscription
        self.br = CvBridge()
        self.img_pub = self.create_publisher(Image, 'debug_image', 1)
        self.idx_pub = self.create_publisher(
            LEDStripProjectionInfoArray, 'led_indices', 10
        )
        self.sub = self.create_subscription(
            Detection3DArray, 'detection_vision_3d', self.cb_detections, 10
        )

        self.sub_scan = self.create_subscription(
            LaserScan,
            'scan',
            self.cb_scan,
            qos_profile=rclpy.qos.qos_profile_sensor_data,
        )

        self._log_configuration()

    def _log_configuration(self) -> None:
        """
        Log the full projector configuration to the ROS2 console.

        Prints:
          - Path to the strips YAML
          - Names of frames (strip & virtual-camera)
          - VirtualStrip summary
          - Virtual perception parameters (distance limits, options)
          - Subscribed and published topic names
        """
        lines = [
            '=== LED-Viz Projector Configuration ===',
            f' strips_config:  {self.cfg_path}',
            f' strip_frame:    {self.cfg.strip_frame}',
            '',
            f' VirtualStrip:   {self.cfg.virtual_strip}',
            '',
            ' Virtual Perception:',
            f'   frame_id:      {self.cfg.vp_frame}',
            f'   min_dist:      {self.cfg.min_d:.2f} m',
            f'   max_dist:      {self.cfg.max_d:.2f} m',
            f'   publish_tf:    {self.cfg.publish_tf}',
            f'   use_centroid:  {self.cfg.use_centroid}',
            f'   offset_t:      {self.cfg.vp_offset_t.tolist()}',
            f'   offset_r:      {self.cfg.vp_offset_r.tolist()}',
            '',
            ' Topics:',
            '   Sub /detection_vision_3d',
            '   Pub /led_indices',
            '   Pub /debug_image',
            '   Pub /led_strip_config',
            '====================================',
        ]
        self.get_logger().info('\n' + '\n'.join(lines))

    def cb_scan(self, msg: LaserScan) -> None:
        """
        Handle incoming LaserScan messages for projecting scan points.

        Projects each laser point into strip coordinates and publishes any
        that lie close to a virtual-strip LED.

        Args
        ----
            msg : LaserScan
                Incoming scan message from a LaserScan topic.

        Returns
        -------
            None
                This function does not return a value.

        """
        norms_list = []

        led_pts, led_ratios, _ = self.cfg.virtual_strip.compute_led_map()
        led_angles = np.array([math.atan2(pt[1], pt[0]) for pt in led_pts])

        origin_v = (0, 0)
        for i, dist in enumerate(msg.ranges):
            angle_sensor = msg.angle_min + i * msg.angle_increment
            pt_sensor = np.array(
                [dist * np.cos(angle_sensor), dist * np.sin(angle_sensor)]
            )

            pt_v = pt_sensor + origin_v
            angle = math.atan2(pt_v[1], pt_v[0])

            if dist > self.cfg.max_d or dist < self.cfg.min_d:
                continue

            dangle = np.abs(led_angles - angle)
            lidx = np.argmin(dangle)
            if dangle[lidx] < 0.1:
                nc = led_ratios[lidx]
                norms_list.append((nc, nc, nc, dist))

        idx_arr = LEDStripProjectionInfoArray()
        idx_arr.header.stamp = msg.header.stamp
        idx_arr.header.frame_id = self.cfg.strip_frame
        for i, (c_n, l_n, r_n, dist) in enumerate(norms_list):
            if c_n is None or dist < self.cfg.min_d or dist > self.cfg.max_d:
                continue
            e = LEDStripProjectionInfo()
            e.header.stamp = msg.header.stamp
            e.header.frame_id = self.cfg.strip_frame
            e.detection_id = i
            e.peak = float(c_n)
            e.start = float(l_n)
            e.stop = float(r_n)
            e.distance = float(dist)
            idx_arr.projection_infos.append(e)
        self.idx_pub.publish(idx_arr)

    def publish_debug_image(self, msg, detection_results) -> None:
        """
        Generate and publish a debug image overlaying strips and detections.

        Args
        ----
            msg
                The Detection3DArray message used for drawing detections.
            detection_results
                List of tuples (DetectionRatios, distance) for visualization.

        Returns
        -------
            None
                This function does not return a value.

        """
        # 1) blank canvas + axes
        im = create_canvas(self.cfg.img_size)
        draw_axes(im, self.cfg.center_px)

        # 2) real strips, virtual strip, and raw red-circle detections
        draw_real_strips(
            im,
            self.cfg.strips,
            self.tfbuf,
            self.cfg.strip_frame,
            self.cfg.vp_frame,
            msg.header.stamp,
            self.cfg.img_size,
            self.cfg.max_d,
            logger=self.get_logger(),
        )
        draw_virtual_strip(
            im,
            self.cfg.virtual_strip,
            self.tfbuf,
            self.cfg.strip_frame,
            self.cfg.vp_frame,
            msg.header.stamp,
            self.cfg.img_size,
            self.cfg.max_d,
            logger=self.get_logger(),
        )
        draw_detections(
            im,
            msg.detections,
            self.tfbuf,
            self.cfg.strip_frame,
            self.cfg.vp_frame,
            msg.header.stamp,
            self.cfg.img_size,
            self.cfg.max_d,
            self.cfg.center_px,
            logger=self.get_logger(),
        )

        # 3) draw the blue dot for each valid centre_ratio
        for ratios, dist in detection_results:
            if ratios is None or ratios.peak is None:
                continue
            c_n = ratios.peak
            # reconstruct the 3D point along the virtual strip
            d_abs = c_n * self.cfg.total_length
            x3, y3, z3 = self.cfg.virtual_strip.interpolate_3d(d_abs)
            p3 = PointStamped()
            p3.header.frame_id = self.cfg.strip_frame
            p3.header.stamp = msg.header.stamp
            p3.point.x, p3.point.y, p3.point.z = x3, y3, z3

            p3c = transform_point(
                self.tfbuf,
                p3,
                self.cfg.vp_frame,
                Duration(seconds=0.1),
                logger=self.get_logger(),
            )
            if p3c is None:
                continue
            pix = world_to_pixel(
                p3c.x, p3c.y, self.cfg.img_size, self.cfg.max_d
            )
            if pix:
                cv2.circle(im, pix, 5, (255, 0, 0), -1)

        # 4) finally, publish the debug image
        img_msg = self.br.cv2_to_imgmsg(im, encoding='bgr8')
        img_msg.header = msg.header
        self.img_pub.publish(img_msg)

    def cb_detections(self, msg: Detection3DArray) -> None:
        """
        Handle incoming Detection3DArray messages.

        Steps:
          1. Transform the virtual-camera origin into the strip frame.
          2. Map each Detection3D to normalized LED ratios via map_detection().
          3. Publish a LEDStripProjectionInfoArray of all detections.
          4. Create debug image

        Args
        ----
            msg : Detection3DArray
                Array of 3D detections from vision.

        Returns
        -------
            None
                This function does not return a value.

        """
        o_cs = PointStamped()
        o_cs.header.frame_id = self.cfg.vp_frame
        o_cs.header.stamp = msg.header.stamp
        o_cs.point.x = o_cs.point.y = o_cs.point.z = 0.0
        cam_strip = transform_point(
            self.tfbuf,
            o_cs,
            self.cfg.strip_frame,
            Duration(seconds=0.1),
            logger=self.get_logger(),
        )
        if cam_strip is None:
            return

        detection_results = []
        for det in msg.detections:
            ratios, dist = map_detection(
                det,
                self.cfg.virtual_strip,
                origin=(cam_strip.x, cam_strip.y),
                min_d=self.cfg.min_d,
                max_d=self.cfg.max_d,
            )
            detection_results.append((ratios, dist))

        idx_arr = LEDStripProjectionInfoArray()
        idx_arr.header.stamp = msg.header.stamp
        idx_arr.header.frame_id = self.cfg.strip_frame
        for i, (ratios, dist) in enumerate(detection_results):
            if ratios is None:
                continue
            e = LEDStripProjectionInfo()
            e.header.stamp = msg.header.stamp
            e.header.frame_id = self.cfg.strip_frame
            e.detection_id = i

            e.peak = float(ratios.peak) if ratios.peak is not None else 0.0
            e.start = float(ratios.start) if ratios.start is not None else 0.0
            e.stop = float(ratios.stop) if ratios.stop is not None else 0.0
            e.distance = float(dist)
            idx_arr.projection_infos.append(e)
        self.idx_pub.publish(idx_arr)

        if self.debug_image:
            self.publish_debug_image(msg, detection_results)


def main(args=None) -> None:
    """
    ROS2 entrypoint for the projector node.

    Initializes rclpy, creates and spins a ProjectorNode,
    and ensures proper shutdown on exit.

    Args
    ----
        args : list, optional
            Command-line arguments (default: None).

    Returns
    -------
        None
            This function does not return a value.

    """
    rclpy.init(args=args)
    node = ProjectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

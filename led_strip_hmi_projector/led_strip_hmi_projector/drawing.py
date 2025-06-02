#!/usr/bin/env python3
"""
Drawing utilities for LED strip visualization.
"""

import math
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np
from geometry_msgs.msg import Point, PointStamped
from rclpy.duration import Duration
from tf2_ros import Buffer

from led_strip_hmi_common.tf_utils import transform_point


def draw_dashed_line(
    img: np.ndarray,
    pt1: Tuple[int, int],
    pt2: Tuple[int, int],
    color: Tuple[int, int, int],
    thickness: int = 1,
    dash_length: int = 10,
    gap_length: int = 5,
) -> None:
    """
    Draw a dashed line between two points on an OpenCV image.

    Args:
        img (np.ndarray): BGR image array.
        pt1 (Tuple[int, int]): Starting pixel coordinates (x1, y1).
        pt2 (Tuple[int, int]): Ending pixel coordinates (x2, y2).
        color (Tuple[int, int, int]): BGR color tuple for the dashes.
        thickness (int): Line thickness in pixels.
        dash_length (int): Length of each dash segment in pixels.
        gap_length (int): Length of the gap between dashes in pixels.
    """
    x1, y1 = pt1
    x2, y2 = pt2
    dx, dy = x2 - x1, y2 - y1
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return
    angle = math.atan2(dy, dx)
    step = dash_length + gap_length
    num = int(dist // step) + 1
    for i in range(num):
        start = i * step
        end = min(start + dash_length, dist)
        sx = int(x1 + math.cos(angle) * start)
        sy = int(y1 + math.sin(angle) * start)
        ex = int(x1 + math.cos(angle) * end)
        ey = int(y1 + math.sin(angle) * end)
        cv2.line(img, (sx, sy), (ex, ey), color, thickness)


def world_to_pixel(
    x: float,
    y: float,
    img_size: int,
    max_distance: float,
) -> Optional[Tuple[int, int]]:
    """
    Project 2D world coordinates into image pixel coordinates.

    Maps ±max_distance in world units to the edges of a square image.

    Args:
        x (float): X coordinate in world space.
        y (float): Y coordinate in world space.
        img_size (int): Width and height of the (square) image in pixels.
        max_distance (float): Maximum world distance corresponding to half the image.

    Returns:
        Optional[Tuple[int, int]]: (px, py) pixel coordinates, or None if out of bounds.
    """
    half = img_size / 2.0
    px = int(half + (x / max_distance) * half)
    py = int(half - (y / max_distance) * half)
    if px < 0 or px >= img_size or py < 0 or py >= img_size:
        return None
    return px, py


def create_canvas(img_size: int, color: Tuple[int,int,int] = (0,0,0)) -> np.ndarray:
    """
    Create a blank BGR image canvas.

    Args:
        img_size (int): Width and height of the (square) image in pixels.
        color (Tuple[int, int, int]): Background BGR color tuple.

    Returns:
        np.ndarray: Blank image of shape (img_size, img_size, 3).
    """
    return np.full((img_size, img_size, 3), color, dtype=np.uint8)


def draw_axes(img: np.ndarray, center_px: int) -> None:
    """
    Draw X and Y coordinate axes on the image.

    Draws a red X-axis to the right and a green Y-axis upward from the center.

    Args:
        img (np.ndarray): BGR image array.
        center_px (int): Pixel index of the image center (both x and y).
    """
    arrow = int(center_px * 0.1)
    cv2.arrowedLine(img, (center_px, center_px),
                    (center_px + arrow, center_px),
                    (0, 0, 255), 2, tipLength=0.3)
    cv2.putText(img, 'X', (center_px + arrow + 5, center_px - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.arrowedLine(img, (center_px, center_px),
                    (center_px, center_px - arrow),
                    (0, 255, 0), 2, tipLength=0.3)
    cv2.putText(img, 'Y', (center_px + 5, center_px - arrow - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)


def draw_real_strips(
    img: np.ndarray,
    strips: Dict[str, Any],
    tfbuf: Buffer,
    strip_frame: str,
    vp_frame: str,
    stamp,
    img_size: int,
    max_d: float,
    logger: Any,
) -> None:
    """
    Overlay the physical LED-strip polygons onto the image.

    Transforms each strip vertex from the strip frame into the virtual-perception
    frame, projects to pixel, and draws solid white polylines.

    Args:
        img (np.ndarray): BGR image array.
        strips (Dict[str, Any]): YAML-defined strips with 'polygon' lists.
        tfbuf (Buffer): TF2 buffer for lookups.
        strip_frame (str): Frame ID of the physical strips.
        vp_frame (str): Virtual-camera frame ID.
        stamp: ROS timestamp for TF queries.
        img_size (int): Debug image size in pixels.
        max_d (float): Maximum world distance for projection.
        logger (Any): Node logger for warnings.
    """
    for name, s in strips.items():
        pts_img = []
        for p in s['polygon']:
            ps = PointStamped()
            ps.header.frame_id = strip_frame
            ps.header.stamp = stamp
            ps.point = Point(x=p[0], y=p[1], z=p[2])

            pc = transform_point(tfbuf, ps, vp_frame,
                                 Duration(seconds=0.1), logger)
            if pc is None:
                continue
            pix = world_to_pixel(pc.x, pc.y, img_size, max_d)
            if pix:
                pts_img.append(pix)

        if len(pts_img) >= 2:
            cv2.polylines(img, [np.array(pts_img, np.int32)],
                          False, (255, 255, 255), 2)


def draw_virtual_strip(
    img: np.ndarray,
    virtual_strip: Any,
    tfbuf: Buffer,
    strip_frame: str,
    vp_frame: str,
    stamp,
    img_size: int,
    max_d: float,
    logger: Any,
) -> None:
    """
    Overlay the virtual-strip segments as dashed gray lines.

    Each segment endpoint is transformed into the virtual-perception frame,
    projected to pixels, then connected with dashes.

    Args:
        img (np.ndarray): BGR image array.
        virtual_strip (Any): VirtualStrip instance.
        tfbuf (Buffer): TF2 buffer for lookups.
        strip_frame (str): Frame ID of the physical strips.
        vp_frame (str): Virtual-camera frame ID.
        stamp: ROS timestamp for TF queries.
        img_size (int): Debug image size in pixels.
        max_d (float): Maximum world distance for projection.
        logger (Any): Node logger for warnings.
    """
    for seg in virtual_strip.segments:
        pts = []
        for pt3d in seg.pts3d:
            ps = PointStamped()
            ps.header.frame_id = strip_frame
            ps.header.stamp = stamp
            ps.point.x, ps.point.y, ps.point.z = pt3d

            pc = transform_point(tfbuf, ps, vp_frame,
                                 Duration(seconds=0.1), logger)
            if pc is None:
                break
            pix = world_to_pixel(pc.x, pc.y, img_size, max_d)
            if pix is None:
                break
            pts.append(pix)

        if len(pts) == 2:
            draw_dashed_line(img, pts[0], pts[1],
                             (127, 127, 127), thickness=1,
                             dash_length=15, gap_length=7)


def draw_detections(
    img: np.ndarray,
    detections,
    tfbuf: Buffer,
    strip_frame: str,
    vp_frame: str,
    stamp,
    img_size: int,
    max_d: float,
    center_px: int,
    logger: Any,
) -> None:
    """
    Draw raw 3D detections as red circles in the virtual-camera frame.

    Transforms each Detection3D center into the virtual-perception frame,
    projects to pixels, and draws a filled red circle scaled by bbox size.

    Args:
        img (np.ndarray): BGR image array.
        detections: List of Detection3D messages.
        tfbuf (Buffer): TF2 buffer for lookups.
        strip_frame (str): Frame ID of the physical strips (unused here).
        vp_frame (str): Virtual-camera frame ID.
        stamp: ROS timestamp for TF queries.
        img_size (int): Debug image size in pixels.
        max_d (float): Maximum world distance for projection.
        center_px (int): Half the image size (for radius scaling).
        logger (Any): Node logger for warnings.
    """
    for det in detections:
        p_vis = PointStamped()
        p_vis.header.frame_id = det.header.frame_id
        p_vis.header.stamp = stamp
        p_vis.point = det.bbox.center.position

        pc = transform_point(tfbuf, p_vis, vp_frame,
                             Duration(seconds=0.1), logger)
        if pc is None:
            continue

        pix = world_to_pixel(pc.x, pc.y, img_size, max_d)
        if not pix:
            continue

        sx, sy = det.bbox.size.x, det.bbox.size.y
        r = max(2, int(((sx + sy) / 2.0) / max_d * center_px))
        cv2.circle(img, pix, r, (0, 0, 255), -1)

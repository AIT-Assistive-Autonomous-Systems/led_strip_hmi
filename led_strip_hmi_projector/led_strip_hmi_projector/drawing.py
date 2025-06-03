#!/usr/bin/env python3
"""Drawing utilities for LED strip visualization."""

import math
from typing import Any, Dict, Optional, Tuple

import cv2

from geometry_msgs.msg import Point, PointStamped
from led_strip_hmi_common.tf_utils import transform_point

import numpy as np

from rclpy.duration import Duration
from tf2_ros import Buffer


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

    Specifically, this function segments the line into alternating dash and gap
    segments and draws them in the given BGR color.

    Parameters
    ----------
    img : np.ndarray
        BGR image array on which to draw.
    pt1 : Tuple[int, int]
        Starting pixel coordinates (x1, y1).
    pt2 : Tuple[int, int]
        Ending pixel coordinates (x2, y2).
    color : Tuple[int, int, int]
        BGR color tuple for the dash segments.
    thickness : int, optional
        Line thickness in pixels. Defaults to 1.
    dash_length : int, optional
        Length of each dash segment in pixels. Defaults to 10.
    gap_length : int, optional
        Length of the gap between dashes in pixels. Defaults to 5.

    Returns
    -------
    None
        This function does not return a value.

    """
    x1, y1 = pt1
    x2, y2 = pt2
    dx, dy = x2 - x1, y2 - y1
    dist = math.hypot(dx, dy)  # Euclidean distance
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

    Maps ±max_distance in world units to the edges of a square image of size
    img_size × img_size. Points outside the ±max_distance range are clamped
    out-of-bounds and return None.

    Parameters
    ----------
    x : float
        X coordinate in world space.
    y : float
        Y coordinate in world space.
    img_size : int
        Width and height (in pixels) of the square image.
    max_distance : float
        Maximum world distance corresponding to half the image.

    Returns
    -------
    Optional[Tuple[int, int]]
        (px, py) pixel coordinates if within bounds, or None if the point
        is outside ±max_distance.

    """
    half = img_size / 2.0
    px = int(half + (x / max_distance) * half)
    py = int(half - (y / max_distance) * half)
    if px < 0 or px >= img_size or py < 0 or py >= img_size:
        return None
    return px, py


def create_canvas(
    img_size: int,
    color: Tuple[int, int, int] = (0, 0, 0),
) -> np.ndarray:
    """
    Create a blank BGR image canvas filled with a background color.

    Parameters
    ----------
    img_size : int
        Width and height (in pixels) of the square image.
    color : Tuple[int, int, int], optional
        Background BGR color tuple. Defaults to black (0, 0, 0).

    Returns
    -------
    np.ndarray
        Blank image of shape (img_size, img_size, 3) filled with 'color'.

    """
    return np.full((img_size, img_size, 3), color, dtype=np.uint8)


def draw_axes(img: np.ndarray, center_px: int) -> None:
    """
    Draw X and Y coordinate axes on the image.

    Draws a red X-axis to the right and a green Y-axis upward from the center
    point. Labels 'X' and 'Y' are also drawn.

    Parameters
    ----------
    img : np.ndarray
        BGR image array on which to draw axes.
    center_px : int
        Pixel index marking the center of the square image.

    Returns
    -------
    None
        This function does not return a value.

    """
    arrow = int(center_px * 0.1)
    cv2.arrowedLine(
        img,
        (center_px, center_px),
        (center_px + arrow, center_px),
        (0, 0, 255),
        2,
        tipLength=0.3,
    )
    cv2.putText(
        img,
        'X',
        (center_px + arrow + 5, center_px - 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 255),
        1,
    )
    cv2.arrowedLine(
        img,
        (center_px, center_px),
        (center_px, center_px - arrow),
        (0, 255, 0),
        2,
        tipLength=0.3,
    )
    cv2.putText(
        img,
        'Y',
        (center_px + 5, center_px - arrow - 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        1,
    )


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

    Each strip vertex is transformed from strip_frame into vp_frame, then
    projected to pixel coordinates and drawn as a white polyline.

    Parameters
    ----------
    img : np.ndarray
        BGR image array on which to draw strip polygons.
    strips : Dict[str, Any]
        Dictionary mapping strip names to polygon vertex lists.
    tfbuf : Buffer
        TF2 buffer for transform lookups.
    strip_frame : str
        Frame ID of the physical LED strips.
    vp_frame : str
        Virtual-camera (perception) frame ID.
    stamp
        ROS timestamp used for TF queries.
    img_size : int
        Size (in pixels) of the debug image.
    max_d : float
        Maximum world distance for projection scaling.
    logger : Any
        Logger instance for warning messages.

    Returns
    -------
    None
        This function does not return a value.

    """
    for name, s in strips.items():
        pts_img = []
        for p in s['polygon']:
            ps = PointStamped()
            ps.header.frame_id = strip_frame
            ps.header.stamp = stamp
            ps.point = Point(x=p[0], y=p[1], z=p[2])

            pc = transform_point(
                tfbuf, ps, vp_frame, Duration(seconds=0.1), logger
            )
            if pc is None:
                continue
            pix = world_to_pixel(pc.x, pc.y, img_size, max_d)
            if pix:
                pts_img.append(pix)

        if len(pts_img) >= 2:
            cv2.polylines(
                img,
                [np.array(pts_img, np.int32)],
                isClosed=False,
                color=(255, 255, 255),
                thickness=2,
            )


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

    Each segment's endpoints are transformed into vp_frame, projected to pixels,
    and connected by dashes via draw_dashed_line().

    Parameters
    ----------
    img : np.ndarray
        BGR image array on which to draw segments.
    virtual_strip : Any
        VirtualStrip instance containing segment geometry.
    tfbuf : Buffer
        TF2 buffer for transform lookups.
    strip_frame : str
        Frame ID of the physical strips.
    vp_frame : str
        Virtual-camera (perception) frame ID.
    stamp
        ROS timestamp used for TF queries.
    img_size : int
        Size (in pixels) of the debug image.
    max_d : float
        Maximum world distance for projection scaling.
    logger : Any
        Logger instance for warning/pruning messages.

    Returns
    -------
    None
        This function does not return a value.

    """
    for seg in virtual_strip.segments:
        pts = []
        for pt3d in seg.pts3d:
            ps = PointStamped()
            ps.header.frame_id = strip_frame
            ps.header.stamp = stamp
            ps.point.x, ps.point.y, ps.point.z = pt3d

            pc = transform_point(
                tfbuf, ps, vp_frame, Duration(seconds=0.1), logger
            )
            if pc is None:
                break
            pix = world_to_pixel(pc.x, pc.y, img_size, max_d)
            if pix is None:
                break
            pts.append(pix)

        if len(pts) == 2:
            draw_dashed_line(
                img,
                pts[0],
                pts[1],
                (127, 127, 127),
                thickness=1,
                dash_length=15,
                gap_length=7,
            )


def draw_detections(
    img: np.ndarray,
    detections,  # List of Detection3D msgs (untyped for brevity)
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

    Each Detection3D center is transformed into vp_frame, projected to pixel
    coordinates, and drawn with a red circle whose radius is scaled by bbox size.

    Parameters
    ----------
    img : np.ndarray
        BGR image array on which to draw detections.
    detections
        Sequence of Detection3D messages to visualize.
    tfbuf : Buffer
        TF2 buffer for transform lookups.
    strip_frame : str
        Frame ID of the physical strips (unused here).
    vp_frame : str
        Virtual-camera (perception) frame ID.
    stamp
        ROS timestamp used for TF queries.
    img_size : int
        Size (in pixels) of the debug image.
    max_d : float
        Maximum world distance for projection scaling.
    center_px : int
        Half the image size (pixel) used to compute circle radius.
    logger : Any
        Logger instance for warning messages.

    Returns
    -------
    None
        This function does not return a value.

    """
    for det in detections:
        p_vis = PointStamped()
        p_vis.header.frame_id = det.header.frame_id
        p_vis.header.stamp = stamp
        p_vis.point = det.bbox.center.position

        pc = transform_point(
            tfbuf, p_vis, vp_frame, Duration(seconds=0.1), logger
        )
        if pc is None:
            continue

        pix = world_to_pixel(pc.x, pc.y, img_size, max_d)
        if not pix:
            continue

        sx, sy = det.bbox.size.x, det.bbox.size.y
        r = max(2, int(((sx + sy) / 2.0) / max_d * center_px))
        cv2.circle(img, pix, r, (0, 0, 255), -1)

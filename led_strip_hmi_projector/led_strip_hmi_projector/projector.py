#!/usr/bin/env python3
"""
LED-strip projecting utilities.

Provides functions to project 3D detections data onto a virtual LED strip,
returning normalized LED indices and ranges.

"""

from dataclasses import dataclass

import math

from typing import Optional, Tuple

from led_strip_hmi_common.virtual_strip import VirtualStrip

import numpy as np

from vision_msgs.msg import Detection3D


@dataclass
class DetectionRatios:
    """
    Container for normalized LED-strip ratios.

    Attributes
    ----------
    start : float
        Normalized position of the leftmost ray projection on the strip [0.0, 1.0].
    peak : float
        Normalized position of the center ray projection on the strip [0.0, 1.0].
    stop : float
        Normalized position of the rightmost ray projection on the strip [0.0, 1.0].

    """

    start: float
    peak: float
    stop: float


def map_detection(
    det: Detection3D,
    vs: VirtualStrip,
    origin: Tuple[float, float],
    min_d: float,
    max_d: float,
) -> Tuple[Optional[DetectionRatios], float]:
    """
    Project a single 3D detection into normalized LED-strip coordinates.

    Casts three rays (center, left, right) from the detection bounding box into the
    2D virtual strip, computing the normalized position along the strip for each ray,
    and returns those ratios along with the detection range.

    Parameters
    ----------
    det : Detection3D
        The input 3D detection message containing bounding-box center and size.
    vs : VirtualStrip
        The VirtualStrip instance used for ray projection and mapping.
    origin : Tuple[float, float]
        (x, y) coordinates of the virtual camera's origin in the strip frame.
    min_d : float
        Minimum valid distance for detections; anything below returns (None, d).
    max_d : float
        Maximum valid distance for detections; anything above returns (None, d).

    Returns
    -------
    Tuple[Optional[DetectionRatios], float]
        A tuple containing:

        - **DetectionRatios** or None:
          - If the detection’s center is within [min_d, max_d], returns a
            `DetectionRatios(start, peak, stop)` where each field is a
            normalized [0.0, 1.0] position along the strip.
          - Returns `None` for ratios if the detection is out of range.
        - **float**:
          The Euclidean distance from the origin to the detection center.

    """
    # Compute detection center in strip XY coordinates
    ds_x = det.bbox.center.position.x - origin[0]
    ds_y = det.bbox.center.position.y - origin[1]
    d = math.hypot(ds_x, ds_y)
    if d < min_d or d > max_d or d < 1e-6:
        # Out of range or too close: return None for ratios and the distance
        return None, d

    # Build orthogonal direction vector for left/right rays
    u = np.array([ds_x, ds_y]) / d
    n = np.array([-u[1], u[0]])
    half_w = det.bbox.size.x / 2.0

    # Project center ray
    _, _, gdp = vs.project_ray2d(ds_x, ds_y, origin=origin)
    # Project left‐offset ray
    _, _, gde = vs.project_ray2d(
        ds_x + n[0] * half_w, ds_y + n[1] * half_w, origin=origin
    )
    # Project right‐offset ray
    _, _, gds = vs.project_ray2d(
        ds_x - n[0] * half_w, ds_y - n[1] * half_w, origin=origin
    )

    ratios = DetectionRatios(start=gds, peak=gdp, stop=gde)
    return (ratios or None), d

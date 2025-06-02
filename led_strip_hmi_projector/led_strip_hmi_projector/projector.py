#!/usr/bin/env python3
"""
LED-strip projecting utilities.

Provides functions to project 3D detections data onto a virtual LED strip,
returning normalized LED indices and ranges.
"""

import math
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
import numpy as np
from vision_msgs.msg import Detection3D
from led_strip_hmi_common.virtual_strip import VirtualStrip

# e.g. {"center": 0.5, "left": 0.45, "right": 0.55}
DetectionRatios = Dict[str, float]


@dataclass
class DetectionRatios:
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

    Args:
        det (Detection3D): The input 3D detection message.
        vs (VirtualStrip): The virtual strip instance used for ray projection.
        origin (Tuple[float, float]): (x, y) of the virtual camera's origin in strip frame.
        min_d (float): Minimum valid distance for detections.
        max_d (float): Maximum valid distance for detections.

    Returns:
        Tuple[Optional[DetectionRatios], float]:
            - A dict mapping keys "center", "left", "right" to normalized positions
              Normalized [0.0, 1.0] along the strip, or None if out of range.
            - The Euclidean distance of the detection from the origin.
    """
    # compute detection center in strip XY
    ds_x = det.bbox.center.position.x - origin[0]
    ds_y = det.bbox.center.position.y - origin[1]
    d = math.hypot(ds_x, ds_y)
    if d < min_d or d > max_d or d < 1e-6:
        return None, d

    # build orthogonal direction for left/right
    u = np.array([ds_x, ds_y]) / d
    n = np.array([-u[1], u[0]])
    half_w = det.bbox.size.x / 2.0

    _, _, gdp = vs.project_ray2d(ds_x, ds_y, origin=origin)
    _, _, gde = vs.project_ray2d(
        ds_x + n[0]*half_w,  ds_y + n[1]*half_w, origin=origin)
    _, _, gds = vs.project_ray2d(
        ds_x - n[0]*half_w,  ds_y - n[1]*half_w, origin=origin) 
    
    

    ratios = DetectionRatios(gds, gdp, gde)
    return (ratios or None), d

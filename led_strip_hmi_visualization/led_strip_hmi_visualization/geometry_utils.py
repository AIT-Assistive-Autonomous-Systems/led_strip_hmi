# geometry_utils.py

import numpy as np
from typing import List, Tuple

def interpolate_polyline(
    polygon: List[Tuple[float, float, float]],
    num_samples: int
) -> np.ndarray:
    """
    Sample num_samples points equally along a 3D polyline.

    Parameters
    ----------
    polygon : list of (x, y, z)
        Consecutive vertices of the polyline.
    num_samples : int
        Number of points to interpolate.

    Returns
    -------
    np.ndarray of shape (num_samples, 3)
    """
    # compute segment lengths
    seg_lengths = []
    total = 0.0
    for (x0,y0,z0), (x1,y1,z1) in zip(polygon, polygon[1:]):
        d = np.hypot(np.hypot(x1-x0, y1-y0), z1-z0)
        seg_lengths.append(d)
        total += d
    if total <= 0 or num_samples <= 0:
        return np.empty((0,3))

    spacing = total / num_samples
    points = []
    for i in range(num_samples):
        target = (i+0.5) * spacing
        acc = 0.0
        for (x0,y0,z0), (x1,y1,z1), seg in zip(polygon, polygon[1:], seg_lengths):
            if acc + seg >= target:
                frac = (target - acc) / seg
                x = x0 + frac*(x1-x0)
                y = y0 + frac*(y1-y0)
                z = z0 + frac*(z1-z0)
                points.append((x,y,z))
                break
            acc += seg
    return np.array(points)

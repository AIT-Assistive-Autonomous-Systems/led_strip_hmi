#!/usr/bin/env python3
"""
TF utility for transforming PointStamped messages.

Provides `transform_point()` to convert a PointStamped from its source frame
into a target frame, returning a geometry_msgs/Point.
"""

from typing import Optional

from geometry_msgs.msg import Point, PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer


def transform_point(
    tf_buffer: Buffer,
    point_stamped: PointStamped,
    target_frame: str,
    timeout: Duration,
    logger: Optional[Node] = None,
) -> Optional[Point]:
    """
    Transform a PointStamped into a Point in the specified target frame.

    Args:
        tf_buffer (Buffer): The TF2 buffer to use for looking up transforms.
        point_stamped (PointStamped): The input stamped point in its source frame.
        target_frame (str): The desired frame to transform the point into.
        timeout (Duration): How long to wait for the necessary transform to become available.
        logger (Optional[Node]): Optional ROS node to log warnings on failure.

    Returns
    -------
        Optional[Point]: The transformed Point in `target_frame`,
                         or None if the transform lookup or application fails.

    """
    try:
        t = tf_buffer.lookup_transform(
            target_frame,
            point_stamped.header.frame_id,
            point_stamped.header.stamp,
            timeout,
        )
        transformed_stamped = do_transform_point(point_stamped, t)
        return transformed_stamped.point
    except Exception as exc:
        if logger:
            logger.warn(
                f'TF transform failed from {point_stamped.header.frame_id} '
                f'to {target_frame}: {exc}'
            )
        return None

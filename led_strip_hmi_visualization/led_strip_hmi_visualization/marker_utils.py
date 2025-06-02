# marker_utils.py

from typing import Tuple, Any, Sequence
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Point


def make_line_marker(
    marker_id: int,
    ns: str,
    points: Sequence[Tuple[float, float, float]],
    header: Header,
    **kwargs: Any
) -> Marker:
    """
    Create a LINE_STRIP marker for the outline of an LED strip.

    Parameters
    ----------
    marker_id : int
        Unique marker ID.
    ns : str
        Marker namespace.
    points : Sequence of (x, y, z)
        3D vertices for the line strip.
    header : Header
        Common ROS message header.

    Returns
    -------
    Marker
        Configured LINE_STRIP marker with points filled.
    """
    m = Marker(**kwargs, action=Marker.ADD, type=Marker.LINE_STRIP)
    m.header = header
    m.ns = ns
    m.id = marker_id
    m.scale.x = 0.01
    m.color.r = 0.2
    m.color.g = 0.8
    m.color.b = 1.0
    m.color.a = 1.0
    m.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in points]
    return m


def make_sphere_marker(
    marker_id: int,
    ns: str,
    position: Tuple[float, float, float],
    diameter: float,
    header: Header,
    rgba: Tuple[float, float, float, float] = (0.5, 0.5, 0.5, 0.25),
    **kwargs: Any
) -> Marker:
    """
    Create a SPHERE marker at a given position.

    Parameters
    ----------
    marker_id : int
        Unique marker ID.
    ns : str
        Marker namespace.
    position : (x, y, z)
        3D coordinates.
    diameter : float
        Sphere diameter.
    header : Header
        Common ROS message header.
    rgba : tuple, optional
        Color and alpha.

    Returns
    -------
    Marker
        Configured SPHERE marker.
    """
    m = Marker(**kwargs, action=Marker.ADD, type=Marker.SPHERE)
    m.header = header
    m.ns = ns
    m.id = marker_id
    m.pose.position.x, m.pose.position.y, m.pose.position.z = position
    m.scale.x = m.scale.y = m.scale.z = diameter
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    return m

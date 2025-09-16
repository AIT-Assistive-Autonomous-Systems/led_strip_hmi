# marker_builders.py

from led_strip_hmi_msgs.msg import LEDStripProjectionInfoArray


from matplotlib import cm


import numpy as np


from rclpy.duration import Duration


from std_msgs.msg import Header


from visualization_msgs.msg import MarkerArray


from .geometry_utils import interpolate_polyline
from .marker_utils import make_line_marker, make_sphere_marker


def build_static_markers(
    cfg,  # ProjectorConfig
    stamp,  # builtin_interfaces.msg.Time
) -> MarkerArray:
    """
    Build markers for static visualization: strip outlines and grey LEDs.

    Uses the `cfg.virtual_strip.strips` definitions (VirtualStrip.Strip instances).

    Parameters
    ----------
    cfg : ProjectorConfig
        Loaded projector configuration.
    stamp : builtin_interfaces.msg.Time
        Timestamp for the markers' headers.

    Returns
    -------
    MarkerArray
        Collection of line and sphere markers representing the static strips.

    """
    m_arr = MarkerArray()
    header = Header(frame_id=cfg.strip_frame, stamp=stamp)
    gid = 0

    # Iterate over VirtualStrip.Strip objects, which have .polygon & .num_leds
    for name, strip in cfg.virtual_strip.strips.items():
        # outline
        gid += 1
        m_arr.markers.append(
            make_line_marker(gid, f'{name}_line', strip.polygon, header)
        )

    for pos, diam in zip(cfg.virtual_strip.leds_xy,
                         cfg.virtual_strip.leds_size):
        gid += 1
        m_arr.markers.append(
            make_sphere_marker(
                gid,
                f'{name}_led2',
                pos,
                diam,
                header))
    return m_arr


def build_dynamic_markers(
    cfg,  # ProjectorConfig
    msg: LEDStripProjectionInfoArray,
    stamp,  # builtin_interfaces.msg.Time
) -> tuple[MarkerArray, MarkerArray, MarkerArray]:
    """
    Build markers for dynamic visualization: continuous highlights and discrete LEDs.

    Parameters
    ----------
    cfg : ProjectorConfig
        Loaded projector configuration.
    msg : LEDStripProjectionInfoArray
        Incoming projection info messages.
    stamp : builtin_interfaces.msg.Time
        Timestamp for the markers' headers.

    Returns
    -------
    highlight_arr, discrete_arr : tuple of MarkerArray
        First array contains continuous strip highlights; second contains discrete LED highlights.

    """
    highlight_arr = MarkerArray()
    discrete_arr = MarkerArray()
    leds_arr = MarkerArray()
    h_gid = d_gid = 0
    header = Header(frame_id=cfg.strip_frame, stamp=stamp)
    life = Duration(seconds=0, nanoseconds=200_000_000).to_msg()
    diam = 0.1

    # Define color tuples as floats
    colors = {
        'peak': (1.0, 0.0, 0.0, 1.0),
        'start': (0.0, 0.0, 1.0, 1.0),
        'stop': (0.0, 1.0, 0.0, 1.0),
    }

    projection_infos = [
        entry for entry in msg.projection_infos if entry.distance is not None
    ]
    projection_infos.sort(key=lambda entry: entry.distance, reverse=True)

    # continuous highlights along the virtual strip
    for entry in msg.projection_infos:
        for attr in ('peak', 'start', 'stop'):
            ratio = getattr(entry, attr, None)
            if ratio is None:
                continue
            d_abs = ratio * cfg.total_length
            pos = cfg.virtual_strip.interpolate_3d(d_abs)
            h_gid += 1
            highlight_arr.markers.append(
                make_sphere_marker(
                    h_gid,
                    'virtual_strip_highlight',
                    pos,
                    diam,
                    header,
                    rgba=colors[attr],
                    lifetime=life,
                )
            )

    # discrete highlights on actual LEDs
    for entry in msg.projection_infos:
        for attr in ('peak', 'start', 'stop'):
            ratio = getattr(entry, attr, None)
            if ratio is None:
                continue
            strip_name, led_idx = cfg.virtual_strip.get_led_at(ratio)
            if strip_name is None:
                continue
            strip = cfg.virtual_strip.strips[strip_name]
            leds = interpolate_polyline(strip.polygon, strip.num_leds)
            pos = leds[led_idx]
            d_gid += 1
            discrete_arr.markers.append(
                make_sphere_marker(
                    d_gid,
                    f'{strip_name}_discrete',
                    pos,
                    diam,
                    header,
                    rgba=colors[attr],
                    lifetime=life,
                )
            )

    leds_xy, leds_ratio = cfg.virtual_strip.leds_xy, cfg.virtual_strip.leds_ratio
    leds_size = cfg.virtual_strip.leds_size
    leds_rgba = np.array([(0.5, 0.5, 0.5, 0.0)
                         for i in range(len(leds_ratio))])

    ncolors = 100
    colormap = cm.get_cmap('cool_r', ncolors)

    def gaussian(x, mean, std_dev):
        if std_dev < 1e-4:
            return np.ones_like(x)
        gauss = (1 / (std_dev * np.sqrt(2 * np.pi))) * np.exp(
            -0.5 * ((x - mean) / std_dev) ** 2
        )
        return gauss / np.max(gauss)

    # discrete highlights on actual LEDs
    for entry in projection_infos:

        min_length = 0.1

        start = entry.start if entry.start is not None else entry.peak - min_length
        stop = entry.stop if entry.stop is not None else entry.peak + min_length

        start = np.clip(start, 0, 1.0)
        stop = np.clip(stop, 0.0, 1.0)

        dleft = max(abs(entry.peak - start), min_length)
        dright = max(abs(stop - entry.peak), min_length)

        ileft = np.searchsorted(leds_ratio, start)
        ipeak = np.searchsorted(leds_ratio, entry.peak)
        iright = np.searchsorted(leds_ratio, stop)

        ileft = np.clip(ileft, 0, len(leds_rgba) - 1)
        ipeak = np.clip(ipeak, 0, len(leds_rgba) - 1)
        iright = np.clip(iright, 0, len(leds_rgba) - 1)

        dratio = (entry.distance - cfg.min_d) / (cfg.max_d - cfg.min_d)
        dratio = np.clip(dratio, 0, 1)
        color = np.array(colormap(dratio))

        def forward_distance(point1, point2, length):
            return (point2 - point1) % length   # number of LEDs in the forward slice

        def set_leds(leds_rgba, color, alpha, start, stop):
            # left wrap
            nalpha = 0
            if start > stop:
                leds_rgba[start:, :] = color
                nalpha = len(leds_rgba) - start
                leds_rgba[start:, 3] = alpha[:nalpha]
                start = 0

            leds_rgba[start:stop, :] = color
            leds_rgba[start:stop, 3] = alpha[nalpha:]

        nleft = forward_distance(ileft, ipeak, len(leds_rgba))
        if nleft > 0:
            xl = np.linspace(start, entry.peak, nleft + 1, endpoint=True)
            gl = gaussian(xl, entry.peak, dleft / 4)[:-1]   # length == nleft
            set_leds(leds_rgba, color, gl, ileft, ipeak)

        nright = forward_distance(ipeak, iright, len(leds_rgba))
        if nright > 0:
            xr = np.linspace(entry.peak, stop, nright + 1, endpoint=True)
            gr = gaussian(xr, entry.peak, dright / 4)[1:]   # length == nright
            set_leds(leds_rgba, color, gr, ipeak, iright)

    leds_rgba = np.clip(leds_rgba, 0.0, 1.0)

    for rgba, pos, diam in zip(leds_rgba, leds_xy, leds_size):
        d_gid += 1
        leds_arr.markers.append(
            make_sphere_marker(
                d_gid,
                'leds_discrete',
                pos,
                diam,
                header,
                rgba=rgba,
                lifetime=life,
            )
        )

    return highlight_arr, discrete_arr, leds_arr

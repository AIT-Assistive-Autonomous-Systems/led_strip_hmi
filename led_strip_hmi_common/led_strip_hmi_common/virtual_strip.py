#!/usr/bin/env python3
"""Module for virtual LED strip geometry and mapping."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from numpy.typing import NDArray
import yaml


@dataclass
class Segment:
    """Represents a 2-point segment of an LED strip or a gap."""

    strip: str
    frame_id: str
    pts: NDArray[np.float64]  # 2D points: shape (2, 2)
    pts3d: NDArray[np.float64]  # 3D points: shape (2, 3)
    length: float = 0.0
    start: float = 0.0
    gap: bool = False


@dataclass
class Strip:
    """Configuration and computed segments for an LED strip."""

    name: str
    polygon: List[List[float]]  # list of [x, y, z]
    num_leds: int
    segments: List[Segment]
    start: float
    length: float


class VirtualStrip:
    """Virtual representation of multiple LED strips for projection and mapping."""

    def __init__(self, yaml_path: str) -> None:
        """
        Load strip configuration and build segment mappings.

        Args:
            yaml_path: Path to the YAML file defining strips.

        Raises
        ------
            FileNotFoundError: If the YAML file does not exist.
            KeyError: If required configuration keys are missing.

        """
        config_path = Path(yaml_path)
        if not config_path.is_file():
            raise FileNotFoundError(
                f'Cannot find configuration file: {yaml_path}')

        with config_path.open('r') as file:
            cfg = yaml.safe_load(file)

        try:
            self.order: List[str] = cfg['order']
            strips_cfg: Dict[str, Any] = cfg['strips']
            self.frame_id: str = cfg['strip_frame_id']
        except KeyError as exc:
            raise KeyError(f'Missing required configuration key: {exc}')

        self.strips: Dict[str, Strip] = {
            name: Strip(
                name=name,
                polygon=strips_cfg[name]['polygon'],
                num_leds=strips_cfg[name]['num_leds'],
                segments=[],
                start=0,
                length=0,
            )
            for name in self.order
        }

        self.closed: bool = cfg.get('closed', False)
        self.segments: List[Segment] = []
        cumulative_length: float = 0.0

        # Build segments and gaps
        for idx, name in enumerate(self.order):
            strip = self.strips[name]
            strip.start = cumulative_length
            polygon = strip.polygon

            for start_pt, end_pt in zip(polygon[:-1], polygon[1:]):
                pts2d = np.array([start_pt[:2], end_pt[:2]], dtype=float)
                pts3d = np.array([start_pt, end_pt], dtype=float)
                segment = Segment(
                    strip=name, frame_id=self.frame_id, pts=pts2d, pts3d=pts3d
                )
                segment.length = float(np.linalg.norm(pts2d[1] - pts2d[0]))
                segment.start = cumulative_length
                cumulative_length += segment.length

                strip.segments.append(segment)
                self.segments.append(segment)

            strip.length = cumulative_length - strip.start

            # Add gap segment if needed
            next_name = (
                self.order[(idx + 1) % len(self.order)]
                if self.closed
                else self.order[idx + 1]
                if idx + 1 < len(self.order)
                else None
            )
            if next_name:
                end_pt = polygon[-1]
                start_next = self.strips[next_name].polygon[0]
                if not np.allclose(end_pt, start_next):
                    pts2d = np.array([end_pt[:2], start_next[:2]], dtype=float)
                    pts3d = np.array([end_pt, start_next], dtype=float)
                    gap_segment = Segment(
                        strip=f'__gap_{name}_to_{next_name}',
                        frame_id=self.frame_id,
                        pts=pts2d,
                        pts3d=pts3d,
                        gap=True,
                    )
                    gap_segment.length = float(
                        np.linalg.norm(pts2d[1] - pts2d[0]))
                    gap_segment.start = cumulative_length
                    cumulative_length += gap_segment.length

                    strip.segments.append(gap_segment)
                    self.segments.append(gap_segment)

        # Cumulative lengths and total length
        self.cum_lengths: List[float] = [0.0] + [
            seg.start + seg.length for seg in self.segments
        ]
        self.total_length: float = cumulative_length

        self.leds_xy, self.leds_ratio, self.leds_size = self.compute_led()

        self.physical_strips = {}
        if 'physical_strip' in cfg:
            for entry in cfg['physical_strip']:
                for phys_id, strip_names in entry.items():
                    segments = []
                    idx = 0
                    while idx < len(self.segments):
                        seg = self.segments[idx]
                        start_ratio = seg.start / self.total_length \
                            if self.total_length > 0 else 0.0
                        stop_ratio = (seg.start + seg.length) / \
                            self.total_length if self.total_length > 0 else 0.0
                        if seg.gap:
                            segments.append({
                                'name': seg.strip,
                                'num_leds': 0,
                                'start': start_ratio,
                                'stop': stop_ratio
                            })
                            idx += 1
                        elif seg.strip in strip_names:
                            # Merge contiguous segments of the same strip
                            total_length = seg.length
                            seg_start = seg.start
                            seg_stop = seg.start + seg.length
                            j = idx + 1
                            while j < len(self.segments) and not self.segments[j].gap \
                                    and self.segments[j].strip == seg.strip:
                                total_length += self.segments[j].length
                                seg_stop += self.segments[j].length
                                j += 1
                            strip = self.strips[seg.strip]
                            segments.append({
                                'name': seg.strip,
                                'num_leds': strip.num_leds,
                                'start': seg_start / self.total_length,
                                'stop': seg_stop / self.total_length
                            })
                            idx = j
                        else:
                            idx += 1
                    self.physical_strips[phys_id] = segments  # use int key

    def __str__(self) -> str:
        """Return summary of the virtual strip."""
        lines = [
            f'VirtualStrip(total_length={self.total_length:.3f} m, segments={len(self.segments)})',
            f'Strips ({len(self.order)}):',
        ]
        for strip in self.strips.values():
            lines.append(
                f'  • {strip.name}: num_leds={strip.num_leds}, control_points={
                    len(strip.polygon)
                }'
            )
        return '\n'.join(lines)

    def project_ray2d(
        self, dx: float, dy: float, origin: Tuple[float, float] = (0.0, 0.0)
    ) -> Tuple[Optional[int], Optional[float], Optional[float]]:
        """
        Cast a 2D ray and return the first intersection with any segment.

        Args:
            dx: X component of the ray direction.
            dy: Y component of the ray direction.
            origin: (x, y) origin of the ray.

        Returns
        -------
            Tuple of (segment_index, local_t, global_distance),
            or (None, None, None) if no intersection.

        """
        direction = np.array([dx, dy], dtype=float)
        if np.linalg.norm(direction) < 1e-6:
            return None, None, None

        origin_pt = np.array(origin, dtype=float)
        best_k = np.inf
        hit_info: Optional[Tuple[int, float]] = None

        for idx, seg in enumerate(self.segments):
            p1, p2 = seg.pts - origin_pt
            v = p2 - p1
            matrix = np.column_stack((direction, -v))
            if abs(np.linalg.det(matrix)) < 1e-8:
                continue
            k, t = np.linalg.solve(matrix, p1)
            if 0 <= t <= 1 and k >= 0 and k < best_k:
                best_k = k
                hit_info = (idx, t)

        if hit_info is None:
            return None, None, None

        seg_idx, t = hit_info
        dist = self.segments[seg_idx].start + t * self.segments[seg_idx].length
        return seg_idx, t, dist / self.total_length

    def interpolate_3d(self, global_dist: float) -> List[float]:
        """
        Interpolate a point in 3D space along the virtual strip.

        Args:
            global_dist: Global distance along the entire strip.

        Returns
        -------
            [x, y, z] coordinates of the interpolated point.

        """
        d = min(max(global_dist, 0.0), self.total_length)
        for idx, seg in enumerate(self.segments):
            start, end = self.cum_lengths[idx], self.cum_lengths[idx + 1]
            if start <= d <= end:
                t = (d - start) / seg.length if seg.length > 0 else 0.0
                p0, p1 = seg.pts3d
                point = p0 + t * (p1 - p0)
                return point.tolist()
        return self.segments[-1].pts3d[-1].tolist()

    def compute_led(self) -> List[List[float]]:
        """
        Compute 3D coordinates of each LED evenly distributed along each strip.

        Returns
        -------
            A list of [x,y,z] positions for all LEDs.

        """
        led_positions: List[List[float]] = []
        led_ratios: List[float] = []
        led_sizes: List[float] = []
        for strip in self.strips.values():
            pts3d = strip.polygon
            n = strip.num_leds
            seg_lengths = [
                np.linalg.norm(np.array(b, float) - np.array(a, float))
                for a, b in zip(pts3d[:-1], pts3d[1:])
            ]
            total = sum(seg_lengths)

            dled = total / strip.num_leds

            for i in range(n):
                frac = 0.0 if n < 2 else i / (n)
                frac += 1.0 / (2 * n)
                d_req = frac * total
                acc = 0.0
                for j, length in enumerate(seg_lengths):
                    if acc + length >= d_req or j == len(seg_lengths) - 1:
                        p0 = np.array(pts3d[j], float)
                        p1 = np.array(pts3d[j + 1], float)
                        t = (d_req - acc) / max(length, 1e-6)
                        position = (p0 + (t) * (p1 - p0)).tolist()
                        ratio = (strip.start + (i + 0.5) * dled) / \
                            self.total_length
                        led_ratios.append(ratio)
                        led_positions.append(position)
                        led_sizes.append(dled)
                        break
                    acc += length

        return led_positions, led_ratios, led_sizes

    def compute_led_map(self) -> List[List[float]]:
        """Alias for compute_led to match original API."""
        return self.compute_led()

    def compute_led_centroid(self) -> List[float]:
        """
        Compute centroid of all LEDs.

        Returns
        -------
            [x,y,z] centroid coordinates.

        """
        leds = np.array(self.leds_xy, dtype=float)
        centroid = np.mean(leds, axis=0)
        return centroid.tolist()

    def get_led_at(self, ratio: float) -> Tuple[Optional[str], Optional[int]]:
        """
        Map a normalized ratio along the virtual strip to a specific LED index.

        Args:
            ratio: Normalized position [0,1] along the entire strip.

        Returns
        -------
            (strip_name, led_index) or (None, None) if in a gap.

        """
        r = min(max(ratio, 0.0), 1.0)
        D = r * self.total_length
        for idx, seg in enumerate(self.segments):
            start, end = self.cum_lengths[idx], self.cum_lengths[idx + 1]
            if not (start <= D <= end):
                continue
            if seg.gap:
                return None, None
            t_seg = (D - start) / seg.length if seg.length > 1e-8 else 0.0
            d_on_strip = 0.0
            for s in self.segments:
                if s.strip != seg.strip:
                    continue
                if s.start + s.length <= seg.start:
                    d_on_strip += s.length
                elif s.start == seg.start:
                    d_on_strip += t_seg * seg.length
                    break
            strip_len = sum(
                s.length for s in self.segments if s.strip == seg.strip)
            if strip_len < 1e-8:
                return seg.strip, 0
            N = self.strips[seg.strip].num_leds
            frac_strip = d_on_strip / strip_len
            led_idx = int(round(frac_strip * (N - 1)))
            led_idx = max(0, min(led_idx, N - 1))
            return seg.strip, led_idx
        return None, None

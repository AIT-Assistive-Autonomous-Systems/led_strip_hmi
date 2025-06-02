"""
Module for loading and validating projector configuration from YAML files.

This module provides a `ProjectorConfig` dataclass with a factory method to load
and validate configuration parameters for the LED strip HMI projector and visualizer.
"""

from __future__ import annotations

from pathlib import Path
from dataclasses import dataclass
from typing import Any, Dict, Union

import numpy as np

from .virtual_strip import VirtualStrip
from .config_utils import read_yaml, validate_keys, ConfigError


@dataclass
class ProjectorConfig:
    """
    Configuration for LED-strip HMI projector and visualizer.

    Attributes
    ----------
    raw_cfg : Dict[str, Any]
        Parsed YAML configuration.
    strips : Dict[str, Any]
        Physical strip definitions (polygons, number of LEDs).
    strip_frame : str
        TF frame ID of the physical strips.
    virtual_strip : VirtualStrip
        VirtualStrip object for normalized LED indexing.
    total_length : float
        Total length of the virtual strip in meters.
    vp_frame : str
        TF frame ID for the virtual-camera (perception) frame.
    min_d : float
        Minimum valid distance for projections.
    max_d : float
        Maximum valid distance for projections.
    publish_tf : bool
        Whether to broadcast a static transform from strip_frame to vp_frame.
    use_centroid : bool
        Whether to center that static transform on the LED centroid.
    vp_offset_t : np.ndarray
        Translation offset [x, y, z] for the virtual-camera frame.
    vp_offset_r : np.ndarray
        Quaternion [x, y, z, w] rotation offset for the virtual-camera frame.
    img_size : int
        Pixel size of the square debug image canvas.
    center_px : int
        Center pixel index (img_size // 2).
    """
    raw_cfg: Dict[str, Any]
    strips: Dict[str, Any]
    strip_frame: str
    virtual_strip: VirtualStrip
    total_length: float
    vp_frame: str
    min_d: float
    max_d: float
    publish_tf: bool
    use_centroid: bool
    vp_offset_t: np.ndarray
    vp_offset_r: np.ndarray
    img_size: int
    center_px: int

    @classmethod
    def load_from_yaml(
        cls,
        path: Union[str, Path]
    ) -> ProjectorConfig:
        """
        Load and validate configuration from a YAML file.

        Parameters
        ----------
        path : str or Path
            Filesystem path to the YAML configuration file.

        Returns
        -------
        ProjectorConfig
            Fully-initialized configuration object.

        Raises
        ------
        FileNotFoundError
            If the YAML file does not exist.
        ConfigError
            If the YAML is malformed or missing required keys/types.
        """
        config_path = Path(path)
        raw: Dict[str, Any] = read_yaml(config_path)

        # Top-level validation
        required_keys: Dict[str, type] = {
            'strips': dict,
            'strip_frame_id': str,
            'virtual_perception': dict
        }
        validate_keys(raw, required_keys)

        strips: Dict[str, Any] = raw['strips']
        strip_frame: str = raw['strip_frame_id']

        # Create VirtualStrip for normalized indexing
        vs: VirtualStrip = VirtualStrip(config_path)
        total_length: float = vs.total_length

        # Virtual-perception parameters
        vp: Dict[str, Any] = raw['virtual_perception']
        required_vp_keys: Dict[str, Union[type, tuple[type, ...]]] = {
            'frame_id': str,
            'min_distance': (int, float),
            'max_distance': (int, float)
        }
        validate_keys(vp, required_vp_keys)

        vp_frame: str = vp['frame_id']
        min_d: float = float(vp['min_distance'])
        max_d: float = float(vp['max_distance'])
        publish_tf: bool = bool(vp.get('publish_transform', False))
        use_centroid: bool = bool(vp.get('use_led_centroid_as_frame', False))

        # Optional offsets
        offset: Dict[str, Any] = vp.get('offset', {})
        vp_offset_t: np.ndarray = np.array(
            offset.get('translation', [0.0, 0.0, 0.0]), dtype=float
        )
        vp_offset_r: np.ndarray = np.array(
            offset.get('rotation', [0.0, 0.0, 0.0, 1.0]), dtype=float
        )

        # Debug image settings
        img_size: int = int(raw.get('img_size', 1024))
        center_px: int = img_size // 2

        return cls(
            raw_cfg=raw,
            strips=strips,
            strip_frame=strip_frame,
            virtual_strip=vs,
            total_length=total_length,
            vp_frame=vp_frame,
            min_d=min_d,
            max_d=max_d,
            publish_tf=publish_tf,
            use_centroid=use_centroid,
            vp_offset_t=vp_offset_t,
            vp_offset_r=vp_offset_r,
            img_size=img_size,
            center_px=center_px,
        )

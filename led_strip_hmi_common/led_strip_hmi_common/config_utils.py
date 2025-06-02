"""
Module for YAML configuration utilities: reading and validating config files.
"""

from pathlib import Path
from typing import Any, Dict

import yaml


class ConfigError(Exception):
    """Exception raised for errors in configuration loading or validation."""


def read_yaml(path: Path) -> Dict[str, Any]:
    """
    Read and parse a YAML file into a Python dictionary.

    Parameters
    ----------
    path : pathlib.Path
        Path to the YAML file.

    Returns
    -------
    Dict[str, Any]
        Parsed YAML content as a dictionary.

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    ConfigError
        If the YAML content is malformed.
    """
    if not path.is_file():
        raise FileNotFoundError(f"Config file not found: {path}")
    try:
        with path.open('r') as f:
            return yaml.safe_load(f)
    except yaml.YAMLError as e:
        # Explicitly chain the YAML parsing error for full context
        raise ConfigError(f"Error parsing YAML file {path}: {e}") from e


def validate_keys(raw: Dict[str, Any], required_keys: Dict[str, type]) -> None:
    """
    Validate the presence and type of required keys in the raw config.

    Parameters
    ----------
    raw : Dict[str, Any]
        The raw configuration dictionary.
    required_keys : Dict[str, type]
        Mapping of key names to their expected types.

    Raises
    ------
    KeyError
        If a required key is missing.
    ConfigError
        If a key is present but has an incorrect type.
    """
    for key, expected_type in required_keys.items():
        if key not in raw:
            raise KeyError(f"Missing required config key: '{key}'")
        if not isinstance(raw[key], expected_type):
            raise ConfigError(
                f"Invalid type for key '{key}': "
                f"expected {expected_type.__name__}, got {type(raw[key]).__name__}"
            )

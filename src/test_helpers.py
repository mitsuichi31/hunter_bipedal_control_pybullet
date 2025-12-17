"""
Shared test helpers for common robot setup.
"""
from typing import Dict, Tuple
import os

from robot_constants import BASE_HEIGHT, standing_config_copy


def urdf_path() -> str:
    """Return the absolute path to the Hunter URDF."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(script_dir, "../models/urdf/hunter.urdf")


def standing_setup() -> Tuple[str, Dict[str, float], float]:
    """
    Provide URDF path, standing joint config, and base height.

    Returns:
        (urdf_path, standing_config copy, base_height)
    """
    return urdf_path(), standing_config_copy(), BASE_HEIGHT

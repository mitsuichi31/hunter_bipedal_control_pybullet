"""
Standing reference helpers for external-controller experiments.
"""

import numpy as np

from ext_joints import LEG_JOINTS
from robot_constants import STANDING_CONFIG


def standing_q_ref() -> np.ndarray:
    """Return standing joint position reference as a (10,) vector."""
    return np.array([STANDING_CONFIG[j] for j in LEG_JOINTS], dtype=float)


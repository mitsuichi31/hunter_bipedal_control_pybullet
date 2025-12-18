"""
Simple safety checks for external-controller experiments.
"""

from typing import Tuple

import numpy as np


def quat_to_rpy_xyzw(q: np.ndarray) -> Tuple[float, float, float]:
    """
    Convert quaternion (x,y,z,w) to roll/pitch/yaw (radians).
    """
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = float(np.arctan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    pitch = float(np.arcsin(np.clip(sinp, -1.0, 1.0)))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = float(np.arctan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw


def should_abort(
    obs,
    *,
    max_roll: float = 0.7,
    max_pitch: float = 0.7,
    min_base_z: float = 0.12,
    max_omega: float = 20.0,
) -> Tuple[bool, str]:
    """
    Return (abort, reason) based on simple thresholds.
    """
    roll, pitch, _ = quat_to_rpy_xyzw(obs.base_quat_xyzw)
    if abs(roll) > max_roll or abs(pitch) > max_pitch:
        return True, f"tilt too large roll={roll:.3f} pitch={pitch:.3f}"
    if float(obs.base_pos[2]) < min_base_z:
        return True, f"base too low z={obs.base_pos[2]:.3f}"
    omega_norm = float(np.linalg.norm(obs.base_omega))
    if omega_norm > max_omega:
        return True, f"omega too large |w|={omega_norm:.3f}"
    return False, ""


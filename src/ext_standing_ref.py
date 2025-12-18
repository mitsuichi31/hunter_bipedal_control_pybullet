"""
Standing reference helpers for external-controller experiments.
"""

from __future__ import annotations

from typing import Dict, Optional

import numpy as np

from ext_joints import LEG_JOINTS
from robot_constants import get_stance_config


def standing_q_ref() -> np.ndarray:
    """Return standing joint position reference as a (10,) vector."""
    return stance_q_ref("standing")


def stance_joint_targets(
    stance: str,
    crouch_knee: Optional[float] = None,
    crouch_ankle: Optional[float] = None,
) -> Dict[str, float]:
    """
    Return joint position targets for a given stance.

    If stance is crouch_* and crouch_knee/ankle are provided, override those magnitudes.
    """
    cfg = get_stance_config(stance)
    s = str(stance)

    if s.startswith("crouch") and (crouch_knee is not None or crouch_ankle is not None):
        if crouch_knee is not None:
            k = float(crouch_knee)
            if s == "crouch_neg":
                cfg["leg_l3_joint"] = -abs(k)
                cfg["leg_r3_joint"] = -abs(k)
                cfg["leg_l4_joint"] = +abs(k)
                cfg["leg_r4_joint"] = +abs(k)
            else:
                cfg["leg_l3_joint"] = +abs(k)
                cfg["leg_r3_joint"] = +abs(k)
                cfg["leg_l4_joint"] = +abs(k)
                cfg["leg_r4_joint"] = +abs(k)
        if crouch_ankle is not None:
            a = float(crouch_ankle)
            if s == "crouch_neg":
                cfg["leg_l5_joint"] = +abs(a)
                cfg["leg_r5_joint"] = +abs(a)
            else:
                cfg["leg_l5_joint"] = -abs(a)
                cfg["leg_r5_joint"] = -abs(a)
    return cfg


def stance_q_ref(
    stance: str,
    crouch_knee: Optional[float] = None,
    crouch_ankle: Optional[float] = None,
) -> np.ndarray:
    """Return stance joint position reference as a (10,) vector."""
    cfg = stance_joint_targets(stance, crouch_knee=crouch_knee, crouch_ankle=crouch_ankle)
    return np.array([cfg[j] for j in LEG_JOINTS], dtype=float)

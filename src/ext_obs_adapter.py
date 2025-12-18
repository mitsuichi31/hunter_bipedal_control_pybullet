"""
Observation adapter for external controller loops.

Converts the simulator's raw observation dict into a normalized dataclass,
including (10,) joint position/velocity vectors matching LEG_JOINTS order.
"""

from dataclasses import dataclass
from typing import Any, Dict

import numpy as np

from ext_joints import LEG_JOINTS


@dataclass
class ExtObs:
    t: float
    base_pos: np.ndarray
    base_quat_xyzw: np.ndarray
    base_vel: np.ndarray
    base_omega: np.ndarray
    joint_pos: Dict[str, float]
    joint_vel: Dict[str, float]
    q: np.ndarray
    dq: np.ndarray
    contact_forces: Dict[str, np.ndarray]
    foot_pos: Dict[str, np.ndarray]
    raw: Dict[str, Any]


def adapt_obs(raw: Dict[str, Any]) -> ExtObs:
    """
    Adapt raw simulator observations into a controller-friendly structure.
    """
    js = raw["joint_states"]
    joint_pos = {jn: float(pv[0]) for jn, pv in js.items()}
    joint_vel = {jn: float(pv[1]) for jn, pv in js.items()}

    q = np.array([joint_pos[j] for j in LEG_JOINTS], dtype=float)
    dq = np.array([joint_vel[j] for j in LEG_JOINTS], dtype=float)

    return ExtObs(
        t=float(raw["time"]),
        base_pos=np.asarray(raw["base_position"], dtype=float),
        base_quat_xyzw=np.asarray(raw["base_orientation"], dtype=float),
        base_vel=np.asarray(raw["base_velocity"], dtype=float),
        base_omega=np.asarray(raw["base_angular_velocity"], dtype=float),
        joint_pos=joint_pos,
        joint_vel=joint_vel,
        q=q,
        dq=dq,
        contact_forces={k: np.asarray(v, dtype=float) for k, v in raw["contact_forces"].items()},
        foot_pos={k: np.asarray(v, dtype=float) for k, v in raw["foot_positions"].items()},
        raw=raw,
    )


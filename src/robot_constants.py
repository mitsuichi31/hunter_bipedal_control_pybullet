"""
Shared robot constants to keep standing poses and heights consistent across
controllers and tests.
"""

from typing import Dict, Tuple

# Canonical base height for the Hunter straight-leg stance.
BASE_HEIGHT: float = 0.679

# Straight-leg joint targets used for initialization/standing checks.
STANDING_CONFIG: Dict[str, float] = {
    "leg_l1_joint": -0.1,
    "leg_l2_joint": 0.0,
    "leg_l3_joint": 0.0,
    "leg_l4_joint": 0.0,
    "leg_l5_joint": 0.0,
    "leg_r1_joint": 0.1,
    "leg_r2_joint": 0.0,
    "leg_r3_joint": 0.0,
    "leg_r4_joint": 0.0,
    "leg_r5_joint": 0.0,
}

# Foot link joint names (ankle pitch).
FOOT_JOINTS: Tuple[str, str] = ("leg_l5_joint", "leg_r5_joint")


def standing_config_copy() -> Dict[str, float]:
    """Return a copy to avoid accidental mutation of the shared config."""
    return dict(STANDING_CONFIG)

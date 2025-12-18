"""
Shared robot constants to keep standing poses and heights consistent across
controllers and tests.
"""

from __future__ import annotations

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

_CROUCH_KNEE: float = 0.6
_CROUCH_ANKLE: float = 0.3

# Crouch stance candidates to reduce COM height and increase passive stability.
# Note: knee sign can be model-dependent, so we provide both pos/neg variants for sweeps.
CROUCH_CONFIG_POS: Dict[str, float] = {
    "leg_l1_joint": STANDING_CONFIG["leg_l1_joint"],
    "leg_l2_joint": STANDING_CONFIG["leg_l2_joint"],
    "leg_l3_joint": +_CROUCH_KNEE,
    "leg_l4_joint": +_CROUCH_KNEE,
    "leg_l5_joint": -_CROUCH_ANKLE,
    "leg_r1_joint": STANDING_CONFIG["leg_r1_joint"],
    "leg_r2_joint": STANDING_CONFIG["leg_r2_joint"],
    "leg_r3_joint": +_CROUCH_KNEE,
    "leg_r4_joint": +_CROUCH_KNEE,
    "leg_r5_joint": -_CROUCH_ANKLE,
}

CROUCH_CONFIG_NEG: Dict[str, float] = {
    "leg_l1_joint": STANDING_CONFIG["leg_l1_joint"],
    "leg_l2_joint": STANDING_CONFIG["leg_l2_joint"],
    "leg_l3_joint": -_CROUCH_KNEE,
    "leg_l4_joint": +_CROUCH_KNEE,
    "leg_l5_joint": +_CROUCH_ANKLE,
    "leg_r1_joint": STANDING_CONFIG["leg_r1_joint"],
    "leg_r2_joint": STANDING_CONFIG["leg_r2_joint"],
    "leg_r3_joint": -_CROUCH_KNEE,
    "leg_r4_joint": +_CROUCH_KNEE,
    "leg_r5_joint": +_CROUCH_ANKLE,
}

STANCE_CONFIGS: Dict[str, Dict[str, float]] = {
    "standing": STANDING_CONFIG,
    "crouch_pos": CROUCH_CONFIG_POS,
    "crouch_neg": CROUCH_CONFIG_NEG,
}


def get_stance_config(name: str) -> Dict[str, float]:
    """
    Return the selected stance config as a copy.

    Unknown names fall back to "standing".
    """
    cfg = STANCE_CONFIGS.get(str(name), STANDING_CONFIG)
    return dict(cfg)


def standing_config_copy() -> Dict[str, float]:
    """Return a copy to avoid accidental mutation of the shared config."""
    return dict(STANDING_CONFIG)


def stance_config_copy(name: str) -> Dict[str, float]:
    """Alias for get_stance_config for call sites that want explicit intent."""
    return get_stance_config(name)

"""
Joint name lists for external-controller experiments.

These are kept in a small standalone module to avoid importing internal
controller code, and to match the repo's flat-import style.
"""

from typing import List


LEG_JOINTS: List[str] = [
    "leg_l1_joint",
    "leg_l2_joint",
    "leg_l3_joint",
    "leg_l4_joint",
    "leg_l5_joint",
    "leg_r1_joint",
    "leg_r2_joint",
    "leg_r3_joint",
    "leg_r4_joint",
    "leg_r5_joint",
]


"""
Reference manager for COM height and velocity targets.
"""

from dataclasses import dataclass
from typing import Dict


@dataclass
class ReferenceTargets:
    com_height: float
    target_displacement_velocity: float
    target_rotation_velocity: float
    default_joint_state: Dict[str, float]


class ReferenceManager:
    def __init__(self, targets: ReferenceTargets):
        self.targets = targets

    def get_targets(self) -> ReferenceTargets:
        return self.targets

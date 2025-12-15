"""
Simple state filters for base and joint signals.
"""

from dataclasses import dataclass
from typing import Dict, Tuple
import numpy as np


@dataclass
class LowPassFilter:
    alpha: float  # smoothing factor in [0,1], higher = less smoothing
    initialized: bool = False
    prev: np.ndarray = None

    def reset(self):
        self.initialized = False
        self.prev = None

    def update(self, value: np.ndarray) -> np.ndarray:
        if not self.initialized or self.prev is None:
            self.prev = value.copy()
            self.initialized = True
            return value

        filtered = self.alpha * value + (1.0 - self.alpha) * self.prev
        self.prev = filtered
        return filtered


class StateFilter:
    """
    Aggregates filters for base pose/velocity and joints.
    """

    def __init__(self,
                 base_alpha: float = 0.2,
                 joint_alpha: float = 0.2):
        self.base_pos_filter = LowPassFilter(alpha=base_alpha)
        self.base_vel_filter = LowPassFilter(alpha=base_alpha)
        self.joint_pos_filters: Dict[str, LowPassFilter] = {}
        self.joint_vel_filters: Dict[str, LowPassFilter] = {}
        self.joint_alpha = joint_alpha

    def reset(self):
        self.base_pos_filter.reset()
        self.base_vel_filter.reset()
        for f in list(self.joint_pos_filters.values()):
            f.reset()
        for f in list(self.joint_vel_filters.values()):
            f.reset()

    def filter_base(self, position: np.ndarray, velocity: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        pos_f = self.base_pos_filter.update(position)
        vel_f = self.base_vel_filter.update(velocity)
        return pos_f, vel_f

    def filter_joints(self, joint_states: Dict[str, Tuple[float, float]]) -> Dict[str, Tuple[float, float]]:
        filtered = {}
        for name, (pos, vel) in joint_states.items():
            if name not in self.joint_pos_filters:
                self.joint_pos_filters[name] = LowPassFilter(alpha=self.joint_alpha)
            if name not in self.joint_vel_filters:
                self.joint_vel_filters[name] = LowPassFilter(alpha=self.joint_alpha)

            pos_f = self.joint_pos_filters[name].update(np.array([pos]))[0]
            vel_f = self.joint_vel_filters[name].update(np.array([vel]))[0]
            filtered[name] = (pos_f, vel_f)
        return filtered

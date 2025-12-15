"""
Observer that wraps basic filtering and contact estimation.
"""

from dataclasses import dataclass
from typing import Dict, Any
import numpy as np

from estimation.state_filter import StateFilter
from estimation.contact_estimator import ContactEstimator, ContactEstimatorParams


@dataclass
class ObserverConfig:
    base_alpha: float = 0.2
    joint_alpha: float = 0.2
    contact_params: ContactEstimatorParams = ContactEstimatorParams()


class Observer:
    """
    Provides filtered state and contact estimates for controllers.
    """

    def __init__(self, config: ObserverConfig = ObserverConfig()):
        self.config = config
        self.filters = StateFilter(base_alpha=config.base_alpha,
                                   joint_alpha=config.joint_alpha)
        self.contact_estimator = ContactEstimator(config.contact_params)

    def reset(self):
        self.filters.reset()
        self.contact_estimator.reset(num_feet=2)

    def update(self,
               base_pos: np.ndarray,
               base_vel: np.ndarray,
               joint_states: Dict[str, Any],
               contact_forces: Dict[str, Any]) -> Dict[str, Any]:
        pos_f, vel_f = self.filters.filter_base(base_pos, base_vel)
        joints_f = self.filters.filter_joints(joint_states)

        # Expect contact_forces mapping foot name -> np.array([fx, fy, fz])
        ordered = [contact_forces.get('left', np.zeros(3)),
                   contact_forces.get('right', np.zeros(3))]
        normals = self.contact_estimator.extract_normal_forces(ordered)
        contacts = self.contact_estimator.update(normals)

        return {
            "base_position": pos_f,
            "base_velocity": vel_f,
            "joint_states": joints_f,
            "contact_normals": normals,
            "contacts": contacts,
        }

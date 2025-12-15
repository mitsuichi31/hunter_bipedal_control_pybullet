"""
Centroidal MPC placeholder aligned with ROS container workflow.
"""

from dataclasses import dataclass
from typing import Dict, Any
import numpy as np


@dataclass
class CentroidalMPCConfig:
    dt: float
    horizon_steps: int
    control_horizon: int
    weights: Dict[str, float]


class CentroidalMPC:
    """
    Minimal placeholder for centroidal MPC.
    Produces simple constant references until replaced with a full solver.
    """

    def __init__(self, config: CentroidalMPCConfig):
        self.config = config

    def plan(self, state: Dict[str, Any], references: Dict[str, Any]) -> Dict[str, Any]:
        com_height = references.get("com_height", 0.63)
        contacts = references.get("contacts", [True, True])
        # Constant CoM reference and zero forces as placeholder
        com_traj = [np.array([0.0, 0.0, com_height]) for _ in range(self.config.horizon_steps)]
        forces = [np.array([0.0, 0.0, 0.0]) for _ in range(len(contacts))]
        return {
            "com_trajectory": com_traj,
            "contact_forces": forces,
            "contact_schedule": contacts,
        }

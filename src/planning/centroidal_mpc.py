"""
Lightweight centroidal MPC stub aligned with ROS container workflow.

Generates a simple preview trajectory using desired velocity + gait schedule,
and distributes gravity across active contacts for force references.
"""

from dataclasses import dataclass
from typing import Dict, Any, List, Optional
import numpy as np

from planning.gait_schedule import GaitSchedule


@dataclass
class CentroidalMPCConfig:
    dt: float
    horizon_steps: int
    control_horizon: int
    weights: Dict[str, float]


class CentroidalMPC:
    """
    Minimal but usable centroidal MPC approximation.
    Produces a CoM preview trajectory and nominal contact forces based on the
    current gait schedule and target velocity.
    """

    def __init__(self,
                 config: CentroidalMPCConfig,
                 gait_schedule: Optional[GaitSchedule] = None,
                 nominal_mass: float = 12.6):
        self.config = config
        self.gait_schedule = gait_schedule
        self.nominal_mass = nominal_mass

    def plan(self,
             state: Dict[str, Any],
             references: Dict[str, Any],
             dt: float) -> Dict[str, Any]:
        """
        Build a CoM trajectory and nominal forces for the horizon.

        Args:
            state: dict with 'com_position' and 'com_velocity'
            references: dict with 'com_height', 'target_velocity'
            dt: control step (seconds)
        """
        com_pos = np.array(state.get("com_position", np.zeros(3)))
        com_vel = np.array(state.get("com_velocity", np.zeros(3)))
        target_vel = np.array(references.get("target_velocity", np.zeros(3)))
        com_height = references.get("com_height", com_pos[2])

        # Contact schedule: from gait if provided, otherwise all stance
        if self.gait_schedule:
            phase = self.gait_schedule.step(dt)
            contact_schedule = self._phase_to_contacts(phase)
        else:
            contact_schedule = references.get("contacts", [True, True])

        # Preview CoM trajectory using constant target velocity
        com_traj: List[np.ndarray] = []
        for k in range(self.config.horizon_steps):
            t = (k + 1) * self.config.dt
            pos = com_pos + target_vel * t
            pos[2] = com_height
            com_traj.append(pos)

        # Nominal contact forces: distribute gravity across active contacts
        forces: List[np.ndarray] = []
        active_contacts = max(sum(contact_schedule), 1)
        fz = self.nominal_mass * 9.81 / active_contacts
        for contact in contact_schedule:
            if contact:
                forces.append(np.array([0.0, 0.0, fz]))
            else:
                forces.append(np.zeros(3))

        return {
            "com_trajectory": com_traj,
            "contact_forces": forces,
            "contact_schedule": contact_schedule,
        }

    @staticmethod
    def _phase_to_contacts(phase: str) -> List[bool]:
        phase = phase.upper()
        if phase == "L":
            return [True, False]
        if phase == "R":
            return [False, True]
        if phase == "STANCE":
            return [True, True]
        if phase == "FLY":
            return [False, False]
        return [True, True]

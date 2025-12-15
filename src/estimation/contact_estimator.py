"""
Contact estimation with force threshold and hysteresis.
"""

from dataclasses import dataclass
from typing import List
import numpy as np


@dataclass
class ContactEstimatorParams:
    contact_threshold: float = 20.0  # Newtons
    release_threshold: float = 10.0  # Newtons (below this, release contact)


class ContactEstimator:
    """
    Estimates foot contact state using normal force thresholds with hysteresis.
    """

    def __init__(self, params: ContactEstimatorParams = ContactEstimatorParams()):
        self.params = params
        self.state: List[bool] = []

    def reset(self, num_feet: int):
        self.state = [False] * num_feet

    def update(self, normal_forces: List[float]) -> List[bool]:
        if not self.state:
            self.reset(len(normal_forces))

        new_state = []
        for i, fz in enumerate(normal_forces):
            in_contact = self.state[i]
            if in_contact:
                in_contact = fz > self.params.release_threshold
            else:
                in_contact = fz > self.params.contact_threshold
            new_state.append(in_contact)

        self.state = new_state
        return new_state

    @staticmethod
    def extract_normal_forces(contact_forces: List[np.ndarray]) -> List[float]:
        normals = []
        for force in contact_forces:
            if force is None or len(force) < 3:
                normals.append(0.0)
            else:
                normals.append(float(force[2]))
        return normals

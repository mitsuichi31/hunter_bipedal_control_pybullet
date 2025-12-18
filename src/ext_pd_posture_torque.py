"""
Simple PD posture controller that outputs torque-only hybrid commands.
"""

from dataclasses import dataclass

import numpy as np

from ext_joints import LEG_JOINTS


@dataclass
class TorquePD:
    # Defaults tuned for a simple standing smoke test (no gravity compensation).
    kp: float = 200.0
    kd: float = 5.0
    tau_limit: float = 200.0


class PDPostureTorque:
    def __init__(self, q_ref: np.ndarray, gains: TorquePD = TorquePD()):
        assert q_ref.shape == (10,)
        self.q_ref = q_ref.astype(float)
        self.g = gains

    def reset(self, obs) -> None:
        pass

    def step(self, obs):
        tau = self.g.kp * (self.q_ref - obs.q) - self.g.kd * obs.dq
        tau = np.clip(tau, -self.g.tau_limit, self.g.tau_limit)
        return {j: {"mode": "torque", "value": float(t)} for j, t in zip(LEG_JOINTS, tau)}

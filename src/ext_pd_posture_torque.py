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
    def __init__(self, q_ref: np.ndarray, gains: TorquePD = TorquePD(), gravity_comp=None):
        assert q_ref.shape == (10,)
        self.q_ref = q_ref.astype(float)
        self.g = gains
        self.gravity_comp = gravity_comp

    def reset(self, obs) -> None:
        pass

    def step(self, obs):
        tau = self.g.kp * (self.q_ref - obs.q) - self.g.kd * obs.dq
        if self.gravity_comp is not None:
            g_dict = self.gravity_comp.compute_gravity_torques_dict(obs.joint_pos)
            g = np.array([float(g_dict.get(j, 0.0)) for j in LEG_JOINTS], dtype=float)
            tau = tau + g
        tau = np.clip(tau, -self.g.tau_limit, self.g.tau_limit)
        return {j: {"mode": "torque", "value": float(t)} for j, t in zip(LEG_JOINTS, tau)}

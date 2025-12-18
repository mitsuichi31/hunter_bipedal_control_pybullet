from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

import numpy as np

from ext_joints import LEG_JOINTS


@dataclass
class PositionStageGains:
    kp: float = 0.3
    kd: float = 0.1


@dataclass
class TorqueStageGains:
    kp: float = 40.0
    kd: float = 1.5
    tau_limit: float = 60.0


class TwoStagePostureController:
    """
    Stage 1 (warmup): position control to quickly establish posture
    Stage 2: torque PD to maintain posture

    Output format matches HunterSimulation.apply_hybrid_command():
      - position: {"mode":"position","value":..., "kp":..., "kd":...}
      - torque:   {"mode":"torque","value":...}
    """

    def __init__(
        self,
        q_ref: np.ndarray,
        *,
        warmup_seconds: float = 1.0,
        position_gains: PositionStageGains = PositionStageGains(),
        torque_gains: TorqueStageGains = TorqueStageGains(),
    ):
        assert q_ref.shape == (10,)
        self.q_ref = q_ref.astype(float)
        self.warmup_seconds = float(warmup_seconds)
        self.pos_g = position_gains
        self.tau_g = torque_gains
        self._t0: Optional[float] = None

    def reset(self, obs) -> None:
        self._t0 = float(obs.t)

    def _in_warmup(self, t: float) -> bool:
        if self._t0 is None:
            self._t0 = t
        return (t - self._t0) < self.warmup_seconds

    def step(self, obs) -> Dict[str, Any]:
        t = float(obs.t)

        if self._in_warmup(t):
            cmds: Dict[str, Any] = {}
            for j, qd in zip(LEG_JOINTS, self.q_ref):
                cmds[j] = {
                    "mode": "position",
                    "value": float(qd),
                    "kp": float(self.pos_g.kp),
                    "kd": float(self.pos_g.kd),
                }
            return cmds

        tau = self.tau_g.kp * (self.q_ref - obs.q) - self.tau_g.kd * obs.dq
        tau = np.clip(tau, -self.tau_g.tau_limit, self.tau_g.tau_limit)
        return {j: {"mode": "torque", "value": float(tv)} for j, tv in zip(LEG_JOINTS, tau)}


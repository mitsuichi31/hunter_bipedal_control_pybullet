"""
External control loop runner.

Tick -> get observations -> optional controller update -> normalize -> apply -> tick
"""

from typing import Any, Dict, Optional

from ext_normalize import normalize_joint_commands
from ext_obs_adapter import adapt_obs
from ext_safety import quat_to_rpy_xyzw, should_abort
from ext_metrics import compute_metrics

import json
import os
import time as _time


def _ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def _now_tag() -> str:
    return _time.strftime("%Y%m%d_%H%M%S")


def _to_list3(x) -> list:
    return [float(x[0]), float(x[1]), float(x[2])]


def _tau_from_normalized_cmds(norm_cmds: Dict[str, Any]) -> Dict[str, float]:
    """
    Extract tau dict when commands are torque/hybrid.
    For position commands, returns {}.
    """
    tau: Dict[str, float] = {}
    for j, c in norm_cmds.items():
        if isinstance(c, dict):
            mode = c.get("mode", "position")
            if mode == "torque":
                tau[j] = float(c.get("value", 0.0))
            elif mode == "hybrid":
                # Feedforward torque only; energy metric is approximate for hybrid mode.
                tau[j] = float(c.get("torque", 0.0))
    return tau


def run(
    sim,
    controller,
    *,
    seconds: float,
    control_dt: float,
    settle_steps: int = 0,
    log_dir: str = "runs",
    run_name: str = "standing_pd_ext",
    safety: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    Run an external controller against HunterSimulation for a fixed duration.
    """
    raw = sim.get_observations()
    obs = adapt_obs(raw)
    if hasattr(controller, "reset"):
        controller.reset(obs)

    last_control_t = obs.t
    end_t = obs.t + float(seconds)

    steps = 0
    updates = 0
    samples = []

    # Optional settling period after reset (lets contacts stabilize).
    for _ in range(max(0, int(settle_steps))):
        sim.step()
        steps += 1

    abort_info = None
    safety_kwargs = safety or {}

    while True:
        raw = sim.get_observations()
        obs = adapt_obs(raw)

        if obs.t >= end_t:
            break

        abort, reason = should_abort(obs, **safety_kwargs)
        if abort:
            abort_info = {"t": float(obs.t), "reason": reason}
            break

        if (obs.t - last_control_t) >= control_dt:
            joint_cmds = controller.step(obs)
            norm_cmds = normalize_joint_commands(joint_cmds)
            sim.apply_hybrid_command(norm_cmds)
            last_control_t = obs.t
            updates += 1

            r, p, y = quat_to_rpy_xyzw(obs.base_quat_xyzw)
            samples.append(
                {
                    "t": float(obs.t),
                    "control_dt": float(control_dt),
                    "base_pos": _to_list3(obs.base_pos),
                    "base_quat_xyzw": [float(v) for v in obs.base_quat_xyzw],
                    "rpy": [float(r), float(p), float(y)],
                    "base_vel": _to_list3(obs.base_vel),
                    "base_omega": _to_list3(obs.base_omega),
                    "joint_pos": {k: float(v) for k, v in obs.joint_pos.items()},
                    "joint_vel": {k: float(v) for k, v in obs.joint_vel.items()},
                    "contact_forces": {k: _to_list3(v) for k, v in obs.contact_forces.items()},
                    "foot_pos": {k: _to_list3(v) for k, v in obs.foot_pos.items()},
                    "tau": _tau_from_normalized_cmds(norm_cmds),
                }
            )

        sim.step()
        steps += 1

    metrics = compute_metrics(samples)

    result: Dict[str, Any] = {
        "status": "ABORT" if abort_info else "DONE",
        "t": float(obs.t),
        "steps": steps,
        "updates": updates,
        "abort": abort_info,
        "metrics": metrics,
    }

    _ensure_dir(log_dir)
    path = os.path.join(log_dir, f"{run_name}_{_now_tag()}.json")
    payload = {
        "result": result,
        "samples": samples,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    result["log_path"] = path
    return result

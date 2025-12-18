"""
External control loop runner.

Tick -> get observations -> optional controller update -> normalize -> apply -> tick
"""

from typing import Any, Dict

from ext_normalize import normalize_joint_commands
from ext_obs_adapter import adapt_obs
from ext_safety import should_abort


def run(sim, controller, *, seconds: float, control_dt: float) -> Dict[str, Any]:
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

    while True:
        raw = sim.get_observations()
        obs = adapt_obs(raw)

        if obs.t >= end_t:
            break

        abort, reason = should_abort(obs)
        if abort:
            return {"status": "ABORT", "t": obs.t, "reason": reason, "steps": steps, "updates": updates}

        if (obs.t - last_control_t) >= control_dt:
            joint_cmds = controller.step(obs)
            sim.apply_hybrid_command(normalize_joint_commands(joint_cmds))
            last_control_t = obs.t
            updates += 1

        sim.step()
        steps += 1

    return {"status": "DONE", "t": obs.t, "steps": steps, "updates": updates}


"""
External control loop runner.

Tick -> get observations -> optional controller update -> normalize -> apply -> tick
"""

from typing import Any, Dict, Optional, Tuple

from ext_normalize import normalize_joint_commands
from ext_obs_adapter import adapt_obs
from ext_safety import quat_to_rpy_xyzw, should_abort
from ext_metrics import compute_metrics

import math
import json
import os
import time as _time


def _ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def _now_tag() -> str:
    return _time.strftime("%Y%m%d_%H%M%S")


def _to_list3(x) -> list:
    return [float(x[0]), float(x[1]), float(x[2])]


def _q_err_rms(obs, controller) -> Optional[float]:
    """
    RMS joint error to controller.q_ref if available.
    Assumes obs.q is (10,) in ext_joints.LEG_JOINTS order.
    """
    try:
        q_ref = getattr(controller, "q_ref", None)
        q = getattr(obs, "q", None)
        if q_ref is None or q is None:
            return None
        if len(q_ref) != len(q):
            return None
        sse = 0.0
        for a, b in zip(q_ref, q):
            d = float(a) - float(b)
            sse += d * d
        return math.sqrt(sse / float(max(1, len(q_ref))))
    except Exception:
        return None


def _norm2_xy(xy) -> Optional[float]:
    try:
        x = float(xy[0])
        y = float(xy[1])
        return math.sqrt(x * x + y * y)
    except Exception:
        return None


def _extract_contact_and_feet(obs) -> Dict[str, Any]:
    """
    Extract contact/feet features for diagnostics.

    Expected obs fields:
      - obs.contact_forces: {"left": np(3,), "right": np(3,)}  (sum force)
      - obs.foot_pos: {"left": np(3,), "right": np(3,)}
    """
    out: Dict[str, Any] = {}
    try:
        cf = getattr(obs, "contact_forces", None) or {}
        fp = getattr(obs, "foot_pos", None) or {}
        lf = cf.get("left") if isinstance(cf, dict) else None
        rf = cf.get("right") if isinstance(cf, dict) else None
        lpos = fp.get("left") if isinstance(fp, dict) else None
        rpos = fp.get("right") if isinstance(fp, dict) else None

        if lf is not None:
            out["fz_left"] = float(lf[2])
            fxy = _norm2_xy(lf[:2])
            if fxy is not None:
                out["fxy_left"] = float(fxy)
        if rf is not None:
            out["fz_right"] = float(rf[2])
            fxy = _norm2_xy(rf[:2])
            if fxy is not None:
                out["fxy_right"] = float(fxy)

        if lpos is not None:
            out["foot_lx"] = float(lpos[0])
            out["foot_ly"] = float(lpos[1])
            out["foot_lz"] = float(lpos[2])
        if rpos is not None:
            out["foot_rx"] = float(rpos[0])
            out["foot_ry"] = float(rpos[1])
            out["foot_rz"] = float(rpos[2])
        if lpos is not None and rpos is not None:
            out["foot_dy"] = abs(float(lpos[1]) - float(rpos[1]))
    except Exception:
        return out

    return out


def _extract_tau_limit(run_meta: Optional[Dict[str, Any]]) -> Optional[float]:
    """
    Best-effort tau_limit extraction from run_meta (written from YAML in main_simulation).
    Supports:
      meta.controller.torque.tau_limit   (two_stage)
      meta.controller.tau_limit          (flat torque_pd legacy)
    """
    if not isinstance(run_meta, dict):
        return None
    ctrl = run_meta.get("controller", {})
    if not isinstance(ctrl, dict):
        return None

    nested = ctrl.get("torque", {})
    if isinstance(nested, dict) and "tau_limit" in nested:
        try:
            return float(nested.get("tau_limit"))
        except Exception:
            return None

    if "tau_limit" in ctrl:
        try:
            return float(ctrl.get("tau_limit"))
        except Exception:
            return None

    return None


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


def _compute_tau_clip_stats(
    norm_cmds: Dict[str, Any],
    tau_limit: Optional[float],
    eps_ratio: float = 0.98,
) -> Tuple[Optional[float], Optional[float]]:
    """
    Saturation proxy stats:
      - tau_clip_frac: fraction of commanded joints whose |tau| is near the limit (>= eps_ratio*tau_limit)
      - tau_clip_max_ratio: max(|tau|/tau_limit) across joints
    Works with normalized hybrid/torque commands.
    """
    if tau_limit is None or float(tau_limit) <= 0.0:
        return (None, None)

    abs_vals = []
    for _, c in (norm_cmds or {}).items():
        if not isinstance(c, dict):
            continue
        mode = c.get("mode", "position")
        if mode == "torque":
            try:
                abs_vals.append(abs(float(c.get("value", 0.0))))
            except Exception:
                pass
        elif mode == "hybrid":
            try:
                abs_vals.append(abs(float(c.get("torque", 0.0))))
            except Exception:
                pass

    if not abs_vals:
        return (None, None)

    thr = float(eps_ratio) * float(tau_limit)
    clip_count = sum(1 for a in abs_vals if a >= thr)
    frac = float(clip_count) / float(len(abs_vals))
    max_ratio = max(a / float(tau_limit) for a in abs_vals)
    return (frac, max_ratio)


def run(
    sim,
    controller,
    *,
    seconds: float,
    control_dt: float,
    settle_steps: int = 0,
    log_dir: str = "runs",
    run_name: str = "standing_pd_ext",
    safety_cfg: Optional[Dict[str, Any]] = None,
    run_meta: Optional[Dict[str, Any]] = None,
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

    abort_info = None
    # Safety thresholds are passed through to should_abort(obs, **kwargs)
    safety_cfg = safety_cfg or {}

    tau_limit = _extract_tau_limit(run_meta)

    # --- IMPORTANT ---
    # If we "settle" the physics before sending ANY motor command, the robot can fall
    # during settle_steps (e.g., 300 ticks = 0.3s). That makes warmup tuning pointless.
    # So: apply an initial command once before settle.
    try:
        joint_cmds0 = controller.step(obs)
        norm_cmds0 = normalize_joint_commands(joint_cmds0)
        tau_clip_frac0, tau_clip_max_ratio0 = _compute_tau_clip_stats(norm_cmds0, tau_limit=tau_limit, eps_ratio=0.98)
        sim.apply_hybrid_command(norm_cmds0)

        r0, p0, y0 = quat_to_rpy_xyzw(obs.base_quat_xyzw)
        s0 = {
            "t": float(obs.t),
            "control_dt": 0.0,
            "base_pos": _to_list3(obs.base_pos),
            "base_quat_xyzw": [float(v) for v in obs.base_quat_xyzw],
            "rpy": [float(r0), float(p0), float(y0)],
            "base_vel": _to_list3(obs.base_vel),
            "base_omega": _to_list3(obs.base_omega),
            "joint_pos": {k: float(v) for k, v in obs.joint_pos.items()},
            "joint_vel": {k: float(v) for k, v in obs.joint_vel.items()},
            "contact_forces": {k: _to_list3(v) for k, v in obs.contact_forces.items()},
            "foot_pos": {k: _to_list3(v) for k, v in obs.foot_pos.items()},
            "tau": _tau_from_normalized_cmds(norm_cmds0),
            "status": "INIT_CMD",
        }
        s0.update(_extract_contact_and_feet(obs))
        # Contact histogram at reset/initial command (can be empty).
        try:
            hist0 = sim.get_contact_link_histogram()
            top0 = sorted(hist0.items(), key=lambda kv: kv[1], reverse=True)[:6] if hist0 else []
            s0["contact_links_top"] = [f"{k}:{v}" for k, v in top0]
        except Exception:
            s0["contact_links_top"] = []
        qe0 = _q_err_rms(obs, controller)
        if qe0 is not None:
            s0["q_err_rms"] = float(qe0)
        if tau_clip_frac0 is not None:
            s0["tau_clip_frac"] = float(tau_clip_frac0)
        if tau_clip_max_ratio0 is not None:
            s0["tau_clip_max_ratio"] = float(tau_clip_max_ratio0)
        samples.append(s0)
    except Exception:
        # Continue anyway; the main loop will try again.
        pass

    # Optional settling period after reset (lets contacts stabilize).
    for _ in range(max(0, int(settle_steps))):
        sim.step()
        steps += 1

    while True:
        raw = sim.get_observations()
        obs = adapt_obs(raw)

        if obs.t >= end_t:
            break

        abort, reason = should_abort(obs, **safety_cfg)
        if abort:
            abort_info = {"t": float(obs.t), "reason": reason}
            break

        if (obs.t - last_control_t) >= control_dt:
            joint_cmds = controller.step(obs)
            norm_cmds = normalize_joint_commands(joint_cmds)
            tau_clip_frac, tau_clip_max_ratio = _compute_tau_clip_stats(norm_cmds, tau_limit=tau_limit, eps_ratio=0.98)
            sim.apply_hybrid_command(norm_cmds)
            last_control_t = obs.t
            updates += 1

            r, p, y = quat_to_rpy_xyzw(obs.base_quat_xyzw)
            s = {
                "status": "RUN",
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
            s.update(_extract_contact_and_feet(obs))
            # Link contact histogram (top-K), for debugging "not touching with feet"
            try:
                hist = sim.get_contact_link_histogram()
                top = sorted(hist.items(), key=lambda kv: kv[1], reverse=True)[:6] if hist else []
                s["contact_links_top"] = [f"{k}:{v}" for k, v in top]
            except Exception:
                s["contact_links_top"] = []
            qe = _q_err_rms(obs, controller)
            if qe is not None:
                s["q_err_rms"] = float(qe)
            if tau_clip_frac is not None:
                s["tau_clip_frac"] = float(tau_clip_frac)
            if tau_clip_max_ratio is not None:
                s["tau_clip_max_ratio"] = float(tau_clip_max_ratio)
            samples.append(s)

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
        "meta": run_meta or {},
        "result": result,
        "samples": samples,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    result["log_path"] = path
    return result

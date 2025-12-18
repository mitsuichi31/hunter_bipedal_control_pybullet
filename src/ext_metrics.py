from __future__ import annotations

from typing import Any, Dict, List
import math


def _energy_step(tau: Dict[str, float], dq: Dict[str, float], dt: float) -> float:
    # Simple mechanical power integral: sum |tau * dq| dt
    e = 0.0
    for j, tj in tau.items():
        v = dq.get(j, 0.0)
        e += abs(tj * v) * dt
    return e


def compute_score(metrics: Dict[str, Any], status: str) -> float:
    """
    Common scoring function used by scripts (compare/sweep).

    Design:
      - Primary: maximize survival_time
      - Strong bonus if DONE
      - Small penalties for tilt and energy (energy is approximate unless torque commands logged)
    """
    survival = float(metrics.get("survival_time", 0.0) or 0.0)
    tilt = metrics.get("tilt_max_abs", None)
    energy = metrics.get("energy_abs_tau_dq", None)

    score = survival
    if str(status).upper() == "DONE":
        score += 100.0

    if tilt is not None:
        try:
            score -= 0.5 * float(tilt)
        except Exception:
            pass

    if energy is not None:
        try:
            score -= 1e-3 * float(energy)
        except Exception:
            pass

    return float(score)


def compute_metrics(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Compute lightweight metrics from ext_runner samples (one per control update).
    """
    if not samples:
        return {
            "n": 0,
            "survival_time": 0.0,
            "tilt_max_abs": None,
            "roll_max_abs": None,
            "pitch_max_abs": None,
            "base_z_min": None,
            "energy_abs_tau_dq": None,
            "foot_slip_left": None,
            "foot_slip_right": None,
        }

    clip_fracs: List[float] = []
    clip_ratios: List[float] = []
    for s in samples:
        if not isinstance(s, dict):
            continue
        cf = s.get("tau_clip_frac", None)
        cr = s.get("tau_clip_max_ratio", None)
        try:
            if cf is not None and not math.isnan(float(cf)):
                clip_fracs.append(float(cf))
        except Exception:
            pass
        try:
            if cr is not None and not math.isnan(float(cr)):
                clip_ratios.append(float(cr))
        except Exception:
            pass

    t0 = float(samples[0]["t"])
    t1 = float(samples[-1]["t"])

    roll_max = 0.0
    pitch_max = 0.0
    base_z_min = float("inf")

    # Foot slip: distance between first and last foot positions (simple heuristic).
    fx0 = samples[0]["foot_pos"]
    fx1 = samples[-1]["foot_pos"]
    slip_left = None
    slip_right = None
    if "left" in fx0 and "left" in fx1:
        a = fx0["left"]
        b = fx1["left"]
        slip_left = math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
    if "right" in fx0 and "right" in fx1:
        a = fx0["right"]
        b = fx1["right"]
        slip_right = math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    energy = 0.0
    for s in samples:
        roll = abs(float(s["rpy"][0]))
        pitch = abs(float(s["rpy"][1]))
        roll_max = max(roll_max, roll)
        pitch_max = max(pitch_max, pitch)
        base_z_min = min(base_z_min, float(s["base_pos"][2]))

        tau = s.get("tau", {})
        dq = s.get("joint_vel", {})
        dt = float(s.get("control_dt", 0.0)) or 0.0
        if isinstance(tau, dict) and isinstance(dq, dict) and dt > 0:
            energy += _energy_step(tau, dq, dt)

    tilt_max = max(roll_max, pitch_max)
    metrics: Dict[str, Any] = {
        "n": len(samples),
        "survival_time": t1 - t0,
        "tilt_max_abs": tilt_max,
        "roll_max_abs": roll_max,
        "pitch_max_abs": pitch_max,
        "base_z_min": base_z_min,
        "energy_abs_tau_dq": energy,
        "foot_slip_left": slip_left,
        "foot_slip_right": slip_right,
    }

    if clip_fracs:
        metrics["tau_clip_frac_mean"] = float(sum(clip_fracs) / len(clip_fracs))
        metrics["tau_clip_frac_max"] = float(max(clip_fracs))
    if clip_ratios:
        metrics["tau_clip_max_ratio_max"] = float(max(clip_ratios))

    return metrics

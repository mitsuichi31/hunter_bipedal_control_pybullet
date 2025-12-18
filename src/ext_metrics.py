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
            "roll_abs_max": None,
            "pitch_abs_max": None,
            "base_z_min": None,
            "base_z_min_sampled": None,
            "energy_abs_tau_dq": None,
            "foot_slip_left": None,
            "foot_slip_right": None,
            "q_err_rms_mean": None,
            "q_err_rms_min": None,
            "q_err_rms_max": None,
        }

    clip_fracs: List[float] = []
    clip_ratios: List[float] = []
    q_err_rms_vals: List[float] = []

    # Contact link debug (counts over RUN samples)
    link_samples_total = 0
    contact_any = 0
    foot_contact_hits = 0
    nonfoot_top: Dict[str, int] = {}

    # Contact / feet features
    fz_left: List[float] = []
    fz_right: List[float] = []
    fxy_left: List[float] = []
    fxy_right: List[float] = []
    foot_dy: List[float] = []
    slip_left_steps: List[float] = []
    slip_right_steps: List[float] = []
    last_l_xy = None
    last_r_xy = None

    for s in samples:
        if not isinstance(s, dict):
            continue
        cf = s.get("tau_clip_frac", None)
        cr = s.get("tau_clip_max_ratio", None)
        qe = s.get("q_err_rms", None)
        try:
            if cf is not None and not math.isnan(float(cf)):
                clip_fracs.append(float(cf))
        except Exception:
            pass

        if str(s.get("status", "")).upper() == "RUN":
            link_samples_total += 1
            top = s.get("contact_links_top", None)
            if isinstance(top, list) and top:
                contact_any += 1
                joined = " ".join([str(x) for x in top]).lower()
                if ("foot" in joined) or ("ankle" in joined) or ("l5" in joined) or ("r5" in joined):
                    foot_contact_hits += 1
                else:
                    try:
                        k = str(top[0]).split(":")[0]
                        nonfoot_top[k] = nonfoot_top.get(k, 0) + 1
                    except Exception:
                        pass
        try:
            if cr is not None and not math.isnan(float(cr)):
                clip_ratios.append(float(cr))
        except Exception:
            pass
        try:
            if qe is not None and not math.isnan(float(qe)):
                q_err_rms_vals.append(float(qe))
        except Exception:
            pass

        # Contact force (flat fields preferred, fallback to nested dicts)
        try:
            if "fz_left" in s:
                fz_left.append(float(s["fz_left"]))
            elif "contact_forces" in s and isinstance(s["contact_forces"], dict) and "left" in s["contact_forces"]:
                fz_left.append(float(s["contact_forces"]["left"][2]))
        except Exception:
            pass
        try:
            if "fz_right" in s:
                fz_right.append(float(s["fz_right"]))
            elif "contact_forces" in s and isinstance(s["contact_forces"], dict) and "right" in s["contact_forces"]:
                fz_right.append(float(s["contact_forces"]["right"][2]))
        except Exception:
            pass
        try:
            if "fxy_left" in s:
                fxy_left.append(float(s["fxy_left"]))
            elif "contact_forces" in s and isinstance(s["contact_forces"], dict) and "left" in s["contact_forces"]:
                lf = s["contact_forces"]["left"]
                fxy_left.append(math.sqrt(float(lf[0]) ** 2 + float(lf[1]) ** 2))
        except Exception:
            pass
        try:
            if "fxy_right" in s:
                fxy_right.append(float(s["fxy_right"]))
            elif "contact_forces" in s and isinstance(s["contact_forces"], dict) and "right" in s["contact_forces"]:
                rf = s["contact_forces"]["right"]
                fxy_right.append(math.sqrt(float(rf[0]) ** 2 + float(rf[1]) ** 2))
        except Exception:
            pass

        try:
            if "foot_dy" in s:
                foot_dy.append(float(s["foot_dy"]))
        except Exception:
            pass

        # Slip (step-to-step planar displacement, per foot)
        try:
            if "foot_lx" in s and "foot_ly" in s:
                cur = (float(s["foot_lx"]), float(s["foot_ly"]))
                if last_l_xy is not None:
                    dx = cur[0] - last_l_xy[0]
                    dy = cur[1] - last_l_xy[1]
                    slip_left_steps.append(math.sqrt(dx * dx + dy * dy))
                last_l_xy = cur
            elif "foot_pos" in s and isinstance(s["foot_pos"], dict) and "left" in s["foot_pos"]:
                lf = s["foot_pos"]["left"]
                cur = (float(lf[0]), float(lf[1]))
                if last_l_xy is not None:
                    dx = cur[0] - last_l_xy[0]
                    dy = cur[1] - last_l_xy[1]
                    slip_left_steps.append(math.sqrt(dx * dx + dy * dy))
                last_l_xy = cur
        except Exception:
            pass

        try:
            if "foot_rx" in s and "foot_ry" in s:
                cur = (float(s["foot_rx"]), float(s["foot_ry"]))
                if last_r_xy is not None:
                    dx = cur[0] - last_r_xy[0]
                    dy = cur[1] - last_r_xy[1]
                    slip_right_steps.append(math.sqrt(dx * dx + dy * dy))
                last_r_xy = cur
            elif "foot_pos" in s and isinstance(s["foot_pos"], dict) and "right" in s["foot_pos"]:
                rf = s["foot_pos"]["right"]
                cur = (float(rf[0]), float(rf[1]))
                if last_r_xy is not None:
                    dx = cur[0] - last_r_xy[0]
                    dy = cur[1] - last_r_xy[1]
                    slip_right_steps.append(math.sqrt(dx * dx + dy * dy))
                last_r_xy = cur
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
        "roll_abs_max": roll_max,
        "pitch_abs_max": pitch_max,
        "base_z_min": base_z_min,
        "base_z_min_sampled": base_z_min,
        "energy_abs_tau_dq": energy,
        "foot_slip_left": slip_left,
        "foot_slip_right": slip_right,
    }

    if fz_left and fz_right:
        ml = float(sum(fz_left) / len(fz_left))
        mr = float(sum(fz_right) / len(fz_right))
        denom = max(1e-6, max(abs(ml), abs(mr)))
        metrics["fz_left_mean"] = ml
        metrics["fz_right_mean"] = mr
        metrics["fz_balance_ratio"] = float(min(abs(ml), abs(mr)) / denom)
    if fxy_left:
        metrics["fxy_left_mean"] = float(sum(fxy_left) / len(fxy_left))
        metrics["fxy_left_max"] = float(max(fxy_left))
    if fxy_right:
        metrics["fxy_right_mean"] = float(sum(fxy_right) / len(fxy_right))
        metrics["fxy_right_max"] = float(max(fxy_right))
    if foot_dy:
        metrics["foot_dy_mean"] = float(sum(foot_dy) / len(foot_dy))
        metrics["foot_dy_min"] = float(min(foot_dy))
    if slip_left_steps:
        metrics["slip_left_mean"] = float(sum(slip_left_steps) / len(slip_left_steps))
        metrics["slip_left_max"] = float(max(slip_left_steps))
    if slip_right_steps:
        metrics["slip_right_mean"] = float(sum(slip_right_steps) / len(slip_right_steps))
        metrics["slip_right_max"] = float(max(slip_right_steps))

    if link_samples_total > 0:
        metrics["contact_any_rate"] = float(contact_any) / float(link_samples_total)
        metrics["foot_contact_hit_rate"] = float(foot_contact_hits) / float(link_samples_total)
        if nonfoot_top:
            metrics["nonfoot_contact_top1"] = sorted(nonfoot_top.items(), key=lambda kv: kv[1], reverse=True)[0][0]
        elif contact_any == 0:
            metrics["nonfoot_contact_top1"] = "NO_CONTACT"
        elif foot_contact_hits > 0:
            # Contacts exist and they look like foot/ankle hits.
            metrics["nonfoot_contact_top1"] = "FOOT_CONTACT"
        else:
            # Contacts exist but we couldn't classify them (e.g., unknown naming).
            metrics["nonfoot_contact_top1"] = "CONTACT"

    if clip_fracs:
        metrics["tau_clip_frac_mean"] = float(sum(clip_fracs) / len(clip_fracs))
        metrics["tau_clip_frac_max"] = float(max(clip_fracs))
    if clip_ratios:
        metrics["tau_clip_max_ratio_max"] = float(max(clip_ratios))
    if q_err_rms_vals:
        metrics["q_err_rms_mean"] = float(sum(q_err_rms_vals) / len(q_err_rms_vals))
        metrics["q_err_rms_min"] = float(min(q_err_rms_vals))
        metrics["q_err_rms_max"] = float(max(q_err_rms_vals))

    return metrics

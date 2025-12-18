#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional


def _import_compute_score(repo_root: str):
    """
    Import src/ext_metrics.py without requiring installation.
    """
    import sys

    src_dir = os.path.join(repo_root, "src")
    if src_dir not in sys.path:
        sys.path.insert(0, src_dir)
    from ext_metrics import compute_score  # type: ignore

    return compute_score


@dataclass
class RunSummary:
    path: str
    mtime: float
    status: str
    score: Optional[float]
    grav_on: Optional[bool]
    grav_scale: Optional[float]
    blend_seconds: Optional[float]
    kd_blend_factor: Optional[float]
    pos_kp: Optional[float]
    pos_kd: Optional[float]
    stance: Optional[str]
    crouch_knee: Optional[float]
    crouch_ankle: Optional[float]
    base_height: Optional[float]
    tau_clip_frac_mean: Optional[float]
    tau_clip_frac_max: Optional[float]
    q_err_rms_mean: Optional[float]
    q_err_rms_min: Optional[float]
    survival_time: Optional[float]
    tilt_max_abs: Optional[float]
    roll_abs_max: Optional[float]
    pitch_abs_max: Optional[float]
    fz_balance_ratio: Optional[float]
    fxy_left_max: Optional[float]
    fxy_right_max: Optional[float]
    slip_left_mean: Optional[float]
    slip_right_mean: Optional[float]
    foot_dy_min: Optional[float]
    foot_contact_hit_rate: Optional[float]
    nonfoot_contact_top1: Optional[str]
    contact_any_rate: Optional[float]
    base_z_min: Optional[float]
    energy_abs_tau_dq: Optional[float]
    foot_slip_left: Optional[float]
    foot_slip_right: Optional[float]
    abort_reason: Optional[str]
    safety_max_roll: Optional[float]
    safety_max_pitch: Optional[float]


def _safe_get(d: Dict[str, Any], keys: List[str], default=None):
    cur: Any = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def _load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _summarize_run(path: str, compute_score_fn) -> Optional[RunSummary]:
    try:
        payload = _load_json(path)
    except Exception:
        return None

    result = payload.get("result", {})
    metrics = result.get("metrics", {}) if isinstance(result, dict) else {}
    abort = result.get("abort", None) if isinstance(result, dict) else None
    meta = payload.get("meta", {}) if isinstance(payload, dict) else {}

    st = _safe_get(metrics, ["survival_time"])
    tilt = _safe_get(metrics, ["tilt_max_abs"])
    roll_abs_max = _safe_get(metrics, ["roll_abs_max"])
    pitch_abs_max = _safe_get(metrics, ["pitch_abs_max"])
    zmin = _safe_get(metrics, ["base_z_min"])
    energy = _safe_get(metrics, ["energy_abs_tau_dq"])
    slip_l = _safe_get(metrics, ["foot_slip_left"])
    slip_r = _safe_get(metrics, ["foot_slip_right"])
    fz_balance_ratio = _safe_get(metrics, ["fz_balance_ratio"])
    fxy_left_max = _safe_get(metrics, ["fxy_left_max"])
    fxy_right_max = _safe_get(metrics, ["fxy_right_max"])
    slip_left_mean = _safe_get(metrics, ["slip_left_mean"])
    slip_right_mean = _safe_get(metrics, ["slip_right_mean"])
    foot_dy_min = _safe_get(metrics, ["foot_dy_min"])
    foot_contact_hit_rate = _safe_get(metrics, ["foot_contact_hit_rate"])
    nonfoot_contact_top1 = _safe_get(metrics, ["nonfoot_contact_top1"])
    contact_any_rate = _safe_get(metrics, ["contact_any_rate"])
    clip_mean = _safe_get(metrics, ["tau_clip_frac_mean"])
    clip_max = _safe_get(metrics, ["tau_clip_frac_max"])
    qerr_mean = _safe_get(metrics, ["q_err_rms_mean"])
    qerr_min = _safe_get(metrics, ["q_err_rms_min"])

    reason = None
    if isinstance(abort, dict):
        reason = abort.get("reason")

    score = None
    try:
        score = float(
            compute_score_fn(metrics if isinstance(metrics, dict) else {}, str(result.get("status", "UNKNOWN")))
        )
    except Exception:
        score = None

    grav_on = _safe_get(meta, ["controller", "torque", "use_gravity_comp"], default=None)
    grav_scale = _safe_get(meta, ["controller", "torque", "gravity_scale"], default=None)
    blend_seconds = _safe_get(meta, ["controller", "blend_seconds"], default=None)
    kd_blend_factor = _safe_get(meta, ["controller", "torque", "kd_blend_factor"], default=None)
    pos_kp = _safe_get(meta, ["controller", "position", "kp"], default=None)
    pos_kd = _safe_get(meta, ["controller", "position", "kd"], default=None)
    stance = _safe_get(meta, ["controller", "stance"], default=None)
    crouch_knee = _safe_get(meta, ["controller", "crouch_knee"], default=None)
    crouch_ankle = _safe_get(meta, ["controller", "crouch_ankle"], default=None)
    base_height = _safe_get(meta, ["runner", "base_height"], default=None)
    try:
        grav_on = bool(grav_on) if grav_on is not None else None
    except Exception:
        grav_on = None
    try:
        grav_scale = float(grav_scale) if grav_scale is not None else None
    except Exception:
        grav_scale = None
    try:
        blend_seconds = float(blend_seconds) if blend_seconds is not None else None
    except Exception:
        blend_seconds = None
    try:
        kd_blend_factor = float(kd_blend_factor) if kd_blend_factor is not None else None
    except Exception:
        kd_blend_factor = None
    try:
        pos_kp = float(pos_kp) if pos_kp is not None else None
    except Exception:
        pos_kp = None
    try:
        pos_kd = float(pos_kd) if pos_kd is not None else None
    except Exception:
        pos_kd = None
    try:
        crouch_knee = float(crouch_knee) if crouch_knee is not None else None
    except Exception:
        crouch_knee = None
    try:
        crouch_ankle = float(crouch_ankle) if crouch_ankle is not None else None
    except Exception:
        crouch_ankle = None
    try:
        base_height = float(base_height) if base_height is not None else None
    except Exception:
        base_height = None
    try:
        clip_mean = float(clip_mean) if clip_mean is not None else None
    except Exception:
        clip_mean = None
    try:
        clip_max = float(clip_max) if clip_max is not None else None
    except Exception:
        clip_max = None
    try:
        qerr_mean = float(qerr_mean) if qerr_mean is not None else None
    except Exception:
        qerr_mean = None
    try:
        qerr_min = float(qerr_min) if qerr_min is not None else None
    except Exception:
        qerr_min = None
    try:
        roll_abs_max = float(roll_abs_max) if roll_abs_max is not None else None
    except Exception:
        roll_abs_max = None
    try:
        pitch_abs_max = float(pitch_abs_max) if pitch_abs_max is not None else None
    except Exception:
        pitch_abs_max = None
    try:
        fz_balance_ratio = float(fz_balance_ratio) if fz_balance_ratio is not None else None
    except Exception:
        fz_balance_ratio = None
    try:
        fxy_left_max = float(fxy_left_max) if fxy_left_max is not None else None
    except Exception:
        fxy_left_max = None
    try:
        fxy_right_max = float(fxy_right_max) if fxy_right_max is not None else None
    except Exception:
        fxy_right_max = None
    try:
        slip_left_mean = float(slip_left_mean) if slip_left_mean is not None else None
    except Exception:
        slip_left_mean = None
    try:
        slip_right_mean = float(slip_right_mean) if slip_right_mean is not None else None
    except Exception:
        slip_right_mean = None
    try:
        foot_dy_min = float(foot_dy_min) if foot_dy_min is not None else None
    except Exception:
        foot_dy_min = None
    try:
        foot_contact_hit_rate = float(foot_contact_hit_rate) if foot_contact_hit_rate is not None else None
    except Exception:
        foot_contact_hit_rate = None
    try:
        contact_any_rate = float(contact_any_rate) if contact_any_rate is not None else None
    except Exception:
        contact_any_rate = None
    try:
        nonfoot_contact_top1 = str(nonfoot_contact_top1) if nonfoot_contact_top1 is not None else None
    except Exception:
        nonfoot_contact_top1 = None

    safety_max_roll = _safe_get(meta, ["safety", "max_roll"], default=None)
    safety_max_pitch = _safe_get(meta, ["safety", "max_pitch"], default=None)
    try:
        safety_max_roll = float(safety_max_roll) if safety_max_roll is not None else None
    except Exception:
        safety_max_roll = None
    try:
        safety_max_pitch = float(safety_max_pitch) if safety_max_pitch is not None else None
    except Exception:
        safety_max_pitch = None

    return RunSummary(
        path=path,
        mtime=os.path.getmtime(path),
        status=str(result.get("status", "UNKNOWN")) if isinstance(result, dict) else "UNKNOWN",
        score=score,
        grav_on=grav_on,
        grav_scale=grav_scale,
        blend_seconds=blend_seconds,
        kd_blend_factor=kd_blend_factor,
        pos_kp=pos_kp,
        pos_kd=pos_kd,
        stance=str(stance) if stance is not None else None,
        crouch_knee=crouch_knee,
        crouch_ankle=crouch_ankle,
        base_height=base_height,
        tau_clip_frac_mean=clip_mean,
        tau_clip_frac_max=clip_max,
        q_err_rms_mean=qerr_mean,
        q_err_rms_min=qerr_min,
        survival_time=float(st) if st is not None else None,
        tilt_max_abs=float(tilt) if tilt is not None else None,
        roll_abs_max=roll_abs_max,
        pitch_abs_max=pitch_abs_max,
        fz_balance_ratio=fz_balance_ratio,
        fxy_left_max=fxy_left_max,
        fxy_right_max=fxy_right_max,
        slip_left_mean=slip_left_mean,
        slip_right_mean=slip_right_mean,
        foot_dy_min=foot_dy_min,
        foot_contact_hit_rate=foot_contact_hit_rate,
        nonfoot_contact_top1=nonfoot_contact_top1,
        contact_any_rate=contact_any_rate,
        base_z_min=float(zmin) if zmin is not None else None,
        energy_abs_tau_dq=float(energy) if energy is not None else None,
        foot_slip_left=float(slip_l) if slip_l is not None else None,
        foot_slip_right=float(slip_r) if slip_r is not None else None,
        abort_reason=str(reason) if reason is not None else None,
        safety_max_roll=safety_max_roll,
        safety_max_pitch=safety_max_pitch,
    )


def _find_logs(log_dir: str, prefix: str) -> List[str]:
    if not os.path.isdir(log_dir):
        return []
    out: List[str] = []
    for fn in os.listdir(log_dir):
        if not fn.endswith(".json"):
            continue
        if prefix and not fn.startswith(prefix):
            continue
        out.append(os.path.join(log_dir, fn))
    out.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return out


def _fmt(x: Optional[float], digits: int = 3) -> str:
    if x is None:
        return "-"
    return f"{x:.{digits}f}"


def _fmt_grav_on(x: Optional[bool]) -> str:
    if x is None:
        return "-"
    return "ON" if x else "OFF"


def _fmt_time(ts: float) -> str:
    return datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S")


def _print_table(rows: List[RunSummary], limit: int):
    rows = rows[:limit]
    headers = [
        "rank",
        "time",
        "status",
        "score",
        "grav",
        "g_scale",
        "blend",
        "kd_blend",
        "base_h",
        "stance",
        "cknee",
        "cankle",
        "s_roll",
        "s_pitch",
        "pos_kp",
        "pos_kd",
        "clip_mean",
        "clip_max",
        "qerr_mean",
        "qerr_min",
        "survival_s",
        "tilt_max(rad)",
        "roll_max",
        "pitch_max",
        "fz_bal",
        "fxyLmx",
        "fxyRmx",
        "slipL",
        "slipR",
        "dy_min",
        "foot_hit",
        "ct_any",
        "top_contact",
        "base_z_min",
        "energy",
        "slip_L",
        "slip_R",
        "abort_reason",
        "file",
    ]
    data = []
    for i, r in enumerate(rows, 1):
        data.append(
            [
                str(i),
                _fmt_time(r.mtime),
                r.status,
                _fmt(r.score, 3),
                _fmt_grav_on(r.grav_on),
                _fmt(r.grav_scale, 3),
                _fmt(r.blend_seconds, 3),
                _fmt(r.kd_blend_factor, 3),
                _fmt(r.base_height, 3),
                (r.stance or "-"),
                _fmt(r.crouch_knee, 3),
                _fmt(r.crouch_ankle, 3),
                _fmt(r.safety_max_roll, 3),
                _fmt(r.safety_max_pitch, 3),
                _fmt(r.pos_kp, 2),
                _fmt(r.pos_kd, 2),
                _fmt(r.tau_clip_frac_mean, 3),
                _fmt(r.tau_clip_frac_max, 3),
                _fmt(r.q_err_rms_mean, 3),
                _fmt(r.q_err_rms_min, 3),
                _fmt(r.survival_time, 3),
                _fmt(r.tilt_max_abs, 3),
                _fmt(r.roll_abs_max, 3),
                _fmt(r.pitch_abs_max, 3),
                _fmt(r.fz_balance_ratio, 3),
                _fmt(r.fxy_left_max, 3),
                _fmt(r.fxy_right_max, 3),
                _fmt(r.slip_left_mean, 4),
                _fmt(r.slip_right_mean, 4),
                _fmt(r.foot_dy_min, 3),
                _fmt(r.foot_contact_hit_rate, 2),
                _fmt(r.contact_any_rate, 2),
                (r.nonfoot_contact_top1 or "-"),
                _fmt(r.base_z_min, 3),
                _fmt(r.energy_abs_tau_dq, 3),
                _fmt(r.foot_slip_left, 3),
                _fmt(r.foot_slip_right, 3),
                (r.abort_reason or "-")[:80],
                os.path.basename(r.path),
            ]
        )

    widths = [len(h) for h in headers]
    for row in data:
        for j, cell in enumerate(row):
            widths[j] = max(widths[j], len(cell))

    def line(sep: str = "-") -> str:
        return "+".join(sep * (w + 2) for w in widths)

    print(line("-"))
    print(" | ".join(h.ljust(widths[i]) for i, h in enumerate(headers)))
    print(line("="))
    for row in data:
        print(" | ".join(row[i].ljust(widths[i]) for i in range(len(headers))))
    print(line("-"))


def main():
    ap = argparse.ArgumentParser(description="Compare latest run logs produced by ext_runner.py")
    ap.add_argument("--log-dir", default="runs", help="Directory containing run JSON logs")
    ap.add_argument("--prefix", default="standing_pd_ext", help="Filename prefix to match (e.g., standing_pd_ext)")
    ap.add_argument("--limit", type=int, default=10, help="Show latest N runs")
    ap.add_argument(
        "--sort-by",
        default="mtime",
        choices=["mtime", "survival", "score", "tilt", "energy", "clip", "qerr", "slip"],
        help="Sort criterion (default: newest first)",
    )
    ap.add_argument(
        "--grav",
        default="any",
        choices=["any", "on", "off"],
        help="Filter by gravity compensation flag stored in meta.controller.torque.use_gravity_comp",
    )
    ap.add_argument("--stance", default="any", choices=["any", "standing", "crouch_pos", "crouch_neg"])
    ap.add_argument("--gscale-min", type=float, default=None, help="Filter: gravity_scale >= this")
    ap.add_argument("--gscale-max", type=float, default=None, help="Filter: gravity_scale <= this")
    ap.add_argument("--blend-min", type=float, default=None, help="Filter: blend_seconds >= this")
    ap.add_argument("--blend-max", type=float, default=None, help="Filter: blend_seconds <= this")
    ap.add_argument("--kdblend-min", type=float, default=None, help="Filter: kd_blend_factor >= this")
    ap.add_argument("--kdblend-max", type=float, default=None, help="Filter: kd_blend_factor <= this")
    ap.add_argument("--poskp-min", type=float, default=None, help="Filter: position kp >= this")
    ap.add_argument("--poskp-max", type=float, default=None, help="Filter: position kp <= this")
    ap.add_argument("--poskd-min", type=float, default=None, help="Filter: position kd >= this")
    ap.add_argument("--poskd-max", type=float, default=None, help="Filter: position kd <= this")
    args = ap.parse_args()

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    compute_score_fn = _import_compute_score(repo_root)

    paths = _find_logs(args.log_dir, args.prefix)
    if not paths:
        print(f"No logs found in '{args.log_dir}' with prefix '{args.prefix}'.")
        return

    runs: List[RunSummary] = []
    for p in paths:
        s = _summarize_run(p, compute_score_fn)
        if s is not None:
            runs.append(s)

    if not runs:
        print("Logs were found but none could be parsed.")
        return

    if args.grav != "any":
        want = True if args.grav == "on" else False
        runs = [r for r in runs if r.grav_on is not None and r.grav_on == want]

    if args.stance != "any":
        runs = [r for r in runs if (r.stance or "") == args.stance]

    if args.gscale_min is not None:
        runs = [r for r in runs if r.grav_scale is not None and r.grav_scale >= args.gscale_min]
    if args.gscale_max is not None:
        runs = [r for r in runs if r.grav_scale is not None and r.grav_scale <= args.gscale_max]

    if args.blend_min is not None:
        runs = [r for r in runs if r.blend_seconds is not None and r.blend_seconds >= args.blend_min]
    if args.blend_max is not None:
        runs = [r for r in runs if r.blend_seconds is not None and r.blend_seconds <= args.blend_max]

    if args.kdblend_min is not None:
        runs = [r for r in runs if r.kd_blend_factor is not None and r.kd_blend_factor >= args.kdblend_min]
    if args.kdblend_max is not None:
        runs = [r for r in runs if r.kd_blend_factor is not None and r.kd_blend_factor <= args.kdblend_max]

    if args.poskp_min is not None:
        runs = [r for r in runs if r.pos_kp is not None and r.pos_kp >= args.poskp_min]
    if args.poskp_max is not None:
        runs = [r for r in runs if r.pos_kp is not None and r.pos_kp <= args.poskp_max]
    if args.poskd_min is not None:
        runs = [r for r in runs if r.pos_kd is not None and r.pos_kd >= args.poskd_min]
    if args.poskd_max is not None:
        runs = [r for r in runs if r.pos_kd is not None and r.pos_kd <= args.poskd_max]

    if not runs:
        print("No runs remain after filtering.")
        return

    if args.sort_by == "mtime":
        runs.sort(key=lambda r: r.mtime, reverse=True)
    elif args.sort_by == "survival":
        runs.sort(key=lambda r: (r.survival_time is None, -(r.survival_time or 0.0), -r.mtime))
    elif args.sort_by == "score":
        runs.sort(key=lambda r: (r.score is None, -(r.score or 0.0), -r.mtime))
    elif args.sort_by == "tilt":
        runs.sort(key=lambda r: (r.tilt_max_abs is None, (r.tilt_max_abs or 0.0), -r.mtime))
    elif args.sort_by == "energy":
        runs.sort(key=lambda r: (r.energy_abs_tau_dq is None, (r.energy_abs_tau_dq or 0.0), -r.mtime))
    elif args.sort_by == "clip":
        # Lower saturation is better:
        #   1) clip_mean asc
        #   2) clip_max asc
        #   3) score desc (tie-break)
        #   4) newest last key
        runs.sort(
            key=lambda r: (
                r.tau_clip_frac_mean is None,
                (r.tau_clip_frac_mean or 1e9),
                r.tau_clip_frac_max is None,
                (r.tau_clip_frac_max or 1e9),
                r.score is None,
                -(r.score or 0.0),
                -r.mtime,
            )
        )
    elif args.sort_by == "qerr":
        # Lower tracking error is better; tie-break by score then newest
        runs.sort(
            key=lambda r: (
                r.q_err_rms_mean is None,
                (r.q_err_rms_mean or 1e9),
                r.q_err_rms_min is None,
                (r.q_err_rms_min or 1e9),
                r.score is None,
                -(r.score or 0.0),
                -r.mtime,
            )
        )
    elif args.sort_by == "slip":
        # Lower mean step-to-step slip is better; tie-break by survival then newest
        def _slip_key(r: RunSummary) -> float:
            vals = []
            if r.slip_left_mean is not None:
                vals.append(r.slip_left_mean)
            if r.slip_right_mean is not None:
                vals.append(r.slip_right_mean)
            return max(vals) if vals else 1e9

        runs.sort(key=lambda r: (_slip_key(r), r.survival_time is None, -(r.survival_time or 0.0), -r.mtime))

    _print_table(runs, args.limit)

    best = runs[0]
    print("\nBest (by sort):", os.path.join(args.log_dir, os.path.basename(best.path)))


if __name__ == "__main__":
    main()

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
    survival_time: Optional[float]
    tilt_max_abs: Optional[float]
    base_z_min: Optional[float]
    energy_abs_tau_dq: Optional[float]
    foot_slip_left: Optional[float]
    foot_slip_right: Optional[float]
    abort_reason: Optional[str]


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
    zmin = _safe_get(metrics, ["base_z_min"])
    energy = _safe_get(metrics, ["energy_abs_tau_dq"])
    slip_l = _safe_get(metrics, ["foot_slip_left"])
    slip_r = _safe_get(metrics, ["foot_slip_right"])

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

    return RunSummary(
        path=path,
        mtime=os.path.getmtime(path),
        status=str(result.get("status", "UNKNOWN")) if isinstance(result, dict) else "UNKNOWN",
        score=score,
        grav_on=grav_on,
        grav_scale=grav_scale,
        blend_seconds=blend_seconds,
        kd_blend_factor=kd_blend_factor,
        survival_time=float(st) if st is not None else None,
        tilt_max_abs=float(tilt) if tilt is not None else None,
        base_z_min=float(zmin) if zmin is not None else None,
        energy_abs_tau_dq=float(energy) if energy is not None else None,
        foot_slip_left=float(slip_l) if slip_l is not None else None,
        foot_slip_right=float(slip_r) if slip_r is not None else None,
        abort_reason=str(reason) if reason is not None else None,
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
        "survival_s",
        "tilt_max(rad)",
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
                _fmt(r.survival_time, 3),
                _fmt(r.tilt_max_abs, 3),
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
        choices=["mtime", "survival", "score", "tilt", "energy"],
        help="Sort criterion (default: newest first)",
    )
    ap.add_argument(
        "--grav",
        default="any",
        choices=["any", "on", "off"],
        help="Filter by gravity compensation flag stored in meta.controller.torque.use_gravity_comp",
    )
    ap.add_argument("--gscale-min", type=float, default=None, help="Filter: gravity_scale >= this")
    ap.add_argument("--gscale-max", type=float, default=None, help="Filter: gravity_scale <= this")
    ap.add_argument("--blend-min", type=float, default=None, help="Filter: blend_seconds >= this")
    ap.add_argument("--blend-max", type=float, default=None, help="Filter: blend_seconds <= this")
    ap.add_argument("--kdblend-min", type=float, default=None, help="Filter: kd_blend_factor >= this")
    ap.add_argument("--kdblend-max", type=float, default=None, help="Filter: kd_blend_factor <= this")
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

    _print_table(runs, args.limit)

    best = runs[0]
    print("\nBest (by sort):", os.path.join(args.log_dir, os.path.basename(best.path)))


if __name__ == "__main__":
    main()

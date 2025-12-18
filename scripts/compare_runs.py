#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional


@dataclass
class RunSummary:
    path: str
    mtime: float
    status: str
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


def _summarize_run(path: str) -> Optional[RunSummary]:
    try:
        payload = _load_json(path)
    except Exception:
        return None

    result = payload.get("result", {})
    metrics = result.get("metrics", {}) if isinstance(result, dict) else {}
    abort = result.get("abort", None) if isinstance(result, dict) else None

    st = _safe_get(metrics, ["survival_time"])
    tilt = _safe_get(metrics, ["tilt_max_abs"])
    zmin = _safe_get(metrics, ["base_z_min"])
    energy = _safe_get(metrics, ["energy_abs_tau_dq"])
    slip_l = _safe_get(metrics, ["foot_slip_left"])
    slip_r = _safe_get(metrics, ["foot_slip_right"])

    reason = None
    if isinstance(abort, dict):
        reason = abort.get("reason")

    return RunSummary(
        path=path,
        mtime=os.path.getmtime(path),
        status=str(result.get("status", "UNKNOWN")) if isinstance(result, dict) else "UNKNOWN",
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


def _fmt_time(ts: float) -> str:
    return datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S")


def _print_table(rows: List[RunSummary], limit: int):
    rows = rows[:limit]
    headers = [
        "rank",
        "time",
        "status",
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
        choices=["mtime", "survival", "tilt", "energy"],
        help="Sort criterion (default: newest first)",
    )
    args = ap.parse_args()

    paths = _find_logs(args.log_dir, args.prefix)
    if not paths:
        print(f"No logs found in '{args.log_dir}' with prefix '{args.prefix}'.")
        return

    runs: List[RunSummary] = []
    for p in paths:
        s = _summarize_run(p)
        if s is not None:
            runs.append(s)

    if not runs:
        print("Logs were found but none could be parsed.")
        return

    if args.sort_by == "mtime":
        runs.sort(key=lambda r: r.mtime, reverse=True)
    elif args.sort_by == "survival":
        runs.sort(key=lambda r: (r.survival_time is None, -(r.survival_time or 0.0), -r.mtime))
    elif args.sort_by == "tilt":
        runs.sort(key=lambda r: (r.tilt_max_abs is None, (r.tilt_max_abs or 0.0), -r.mtime))
    elif args.sort_by == "energy":
        runs.sort(key=lambda r: (r.energy_abs_tau_dq is None, (r.energy_abs_tau_dq or 0.0), -r.mtime))

    _print_table(runs, args.limit)

    best = runs[0]
    print("\nBest (by sort):", os.path.join(args.log_dir, os.path.basename(best.path)))


if __name__ == "__main__":
    main()


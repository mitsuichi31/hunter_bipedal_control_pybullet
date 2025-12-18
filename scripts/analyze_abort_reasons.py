#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import re
from collections import Counter
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


def _load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _norm_reason(reason: str) -> str:
    """
    Normalize abort reason strings to improve aggregation.
    Example:
      "tilt too large roll=0.021 pitch=0.703" -> "tilt too large"
    """
    r = (reason or "").strip()
    if not r:
        return "UNKNOWN"
    r = re.sub(r"\s+[a-zA-Z_]+=-?\d+(\.\d+)?", "", r)
    r = re.sub(r"\(.*?\)", "", r).strip()
    return r


def _get(d: Dict[str, Any], path: List[str], default=None):
    cur: Any = d
    for k in path:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


@dataclass
class TrialRow:
    idx: int
    status: str
    success_rate: float
    survival_mean: float
    survival_min: float
    score_mean: float
    grav_on: Optional[bool]
    grav_scale: Optional[float]
    warmup_seconds: Optional[float]
    abort_reasons: List[str]


def _parse_trial(t: Dict[str, Any]) -> TrialRow:
    idx = int(t.get("idx", -1))
    status = str(t.get("status", ""))
    success_rate = float(t.get("success_rate", 0.0) or 0.0)
    survival_mean = float(t.get("survival_mean", 0.0) or 0.0)
    survival_min = float(t.get("survival_min", 0.0) or 0.0)
    score_mean = float(t.get("score_mean", 0.0) or 0.0)

    params = t.get("params", {}) if isinstance(t.get("params", {}), dict) else {}
    grav_on = params.get("use_gravity_comp", None)
    grav_scale = params.get("gravity_scale", None)
    warmup = params.get("warmup_seconds", None)

    try:
        grav_on = bool(grav_on) if grav_on is not None else None
    except Exception:
        grav_on = None
    try:
        grav_scale = float(grav_scale) if grav_scale is not None else None
    except Exception:
        grav_scale = None
    try:
        warmup = float(warmup) if warmup is not None else None
    except Exception:
        warmup = None

    reasons = t.get("abort_reasons", []) or []
    if not isinstance(reasons, list):
        reasons = [str(reasons)]
    reasons = [str(r) for r in reasons if r is not None]

    return TrialRow(
        idx=idx,
        status=status,
        success_rate=success_rate,
        survival_mean=survival_mean,
        survival_min=survival_min,
        score_mean=score_mean,
        grav_on=grav_on,
        grav_scale=grav_scale,
        warmup_seconds=warmup,
        abort_reasons=reasons,
    )


def _print_top(counter: Counter, title: str, limit: int = 10) -> None:
    print(f"\n== {title} ==")
    if not counter:
        print("(none)")
        return
    total = sum(counter.values())
    for k, v in counter.most_common(limit):
        pct = (100.0 * v / total) if total > 0 else 0.0
        print(f"{v:6d}  ({pct:5.1f}%)  {k}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Analyze abort reasons from runs/sweep_summary.json")
    ap.add_argument("--summary", default="runs/sweep_summary.json", help="Path to sweep_summary.json")
    ap.add_argument("--limit", type=int, default=12, help="Top-N reasons to show")
    ap.add_argument("--min-trials", type=int, default=0, help="Require at least this many trials (sanity)")
    args = ap.parse_args()

    if not os.path.exists(args.summary):
        print(f"Not found: {args.summary}")
        return 2

    summary = _load_json(args.summary)
    results = summary.get("results", []) or []
    if not isinstance(results, list):
        print("Invalid sweep_summary.json: 'results' is not a list")
        return 2

    if len(results) < args.min_trials:
        print(f"Too few trials: {len(results)} < {args.min_trials}")
        return 2

    rows = [_parse_trial(t) for t in results if isinstance(t, dict)]
    if not rows:
        print("No trial rows found.")
        return 2

    c_all = Counter()
    for r in rows:
        for reason in r.abort_reasons:
            c_all[_norm_reason(reason)] += 1
    _print_top(c_all, "Abort reasons (all)", limit=args.limit)

    c_on = Counter()
    c_off = Counter()
    for r in rows:
        target = c_on if r.grav_on else c_off
        for reason in r.abort_reasons:
            target[_norm_reason(reason)] += 1
    _print_top(c_off, "Abort reasons (gravity OFF)", limit=args.limit)
    _print_top(c_on, "Abort reasons (gravity ON)", limit=args.limit)

    def best_key(r: TrialRow) -> Tuple[float, float, float, float]:
        return (r.success_rate, r.survival_min, r.score_mean, 0.0)

    rows_sorted = sorted(rows, key=best_key, reverse=True)
    print("\n== Top trials by (success_rate, survival_min, score_mean) ==")
    for r in rows_sorted[: min(10, len(rows_sorted))]:
        grav = "ON" if r.grav_on else "OFF"
        gs = f"{r.grav_scale:.2f}" if r.grav_scale is not None else "-"
        wu = f"{r.warmup_seconds:.2f}" if r.warmup_seconds is not None else "-"
        print(
            f"idx={r.idx:3d}  {r.status:10s}  sr={r.success_rate:.2f}  "
            f"surv_min={r.survival_min:6.3f}  surv_mean={r.survival_mean:6.3f}  "
            f"score_mean={r.score_mean:8.3f}  grav={grav}  gscale={gs}  warmup={wu}"
        )

    if max(r.success_rate for r in rows) <= 0.0:
        top_reason = c_all.most_common(1)[0][0] if c_all else "UNKNOWN"
        print("\n== Hint ==")
        print("All trials have success_rate=0.0 (no DONE).")
        print(f"Most common abort reason (normalized): {top_reason}")
        print("Next steps usually help:")
        print("  - Increase warmup_seconds (e.g., 1.0 -> 2.0) and/or settle_steps")
        print("  - If tilt-related, try higher kd in hold, or slightly lower kp, or gravity_scale sweep")
        print("  - If base_z low, check initial pose / BASE_HEIGHT / stance width")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


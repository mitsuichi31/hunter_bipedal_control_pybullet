```diff
--- /dev/null
+++ b/scripts/analyze_abort_reasons.py
@@ -0,0 +1,245 @@
+#!/usr/bin/env python3
+from __future__ import annotations
+
+import argparse
+import json
+import os
+import re
+from collections import Counter, defaultdict
+from dataclasses import dataclass
+from typing import Any, Dict, List, Optional, Tuple
+
+
+def _load_json(path: str) -> Dict[str, Any]:
+    with open(path, "r", encoding="utf-8") as f:
+        return json.load(f)
+
+
+def _norm_reason(reason: str) -> str:
+    """
+    Normalize abort reason strings to improve aggregation.
+    Example:
+      "tilt too large roll=0.021 pitch=0.703" -> "tilt too large"
+    """
+    r = (reason or "").strip()
+    if not r:
+        return "UNKNOWN"
+    # strip numeric suffixes like "x=..., y=..."
+    r = re.sub(r"\s+[a-zA-Z_]+=-?\d+(\.\d+)?", "", r)
+    # strip parentheses payloads
+    r = re.sub(r"\(.*?\)", "", r).strip()
+    return r
+
+
+def _get(d: Dict[str, Any], path: List[str], default=None):
+    cur: Any = d
+    for k in path:
+        if not isinstance(cur, dict) or k not in cur:
+            return default
+        cur = cur[k]
+    return cur
+
+
+@dataclass
+class TrialRow:
+    idx: int
+    status: str
+    success_rate: float
+    survival_mean: float
+    survival_min: float
+    score_mean: float
+    grav_on: Optional[bool]
+    grav_scale: Optional[float]
+    warmup_seconds: Optional[float]
+    abort_reasons: List[str]
+
+
+def _parse_trial(t: Dict[str, Any]) -> TrialRow:
+    idx = int(t.get("idx", -1))
+    status = str(t.get("status", ""))
+    success_rate = float(t.get("success_rate", 0.0) or 0.0)
+    survival_mean = float(t.get("survival_mean", 0.0) or 0.0)
+    survival_min = float(t.get("survival_min", 0.0) or 0.0)
+    score_mean = float(t.get("score_mean", 0.0) or 0.0)
+
+    params = t.get("params", {}) if isinstance(t.get("params", {}), dict) else {}
+    grav_on = params.get("use_gravity_comp", None)
+    grav_scale = params.get("gravity_scale", None)
+    warmup = params.get("warmup_seconds", None)
+
+    try:
+        grav_on = bool(grav_on) if grav_on is not None else None
+    except Exception:
+        grav_on = None
+    try:
+        grav_scale = float(grav_scale) if grav_scale is not None else None
+    except Exception:
+        grav_scale = None
+    try:
+        warmup = float(warmup) if warmup is not None else None
+    except Exception:
+        warmup = None
+
+    reasons = t.get("abort_reasons", []) or []
+    if not isinstance(reasons, list):
+        reasons = [str(reasons)]
+    reasons = [str(r) for r in reasons if r is not None]
+
+    return TrialRow(
+        idx=idx,
+        status=status,
+        success_rate=success_rate,
+        survival_mean=survival_mean,
+        survival_min=survival_min,
+        score_mean=score_mean,
+        grav_on=grav_on,
+        grav_scale=grav_scale,
+        warmup_seconds=warmup,
+        abort_reasons=reasons,
+    )
+
+
+def _print_top(counter: Counter, title: str, limit: int = 10) -> None:
+    print(f"\n== {title} ==")
+    if not counter:
+        print("(none)")
+        return
+    total = sum(counter.values())
+    for k, v in counter.most_common(limit):
+        pct = (100.0 * v / total) if total > 0 else 0.0
+        print(f"{v:6d}  ({pct:5.1f}%)  {k}")
+
+
+def main() -> int:
+    ap = argparse.ArgumentParser(description="Analyze abort reasons from runs/sweep_summary.json")
+    ap.add_argument("--summary", default="runs/sweep_summary.json", help="Path to sweep_summary.json")
+    ap.add_argument("--limit", type=int, default=12, help="Top-N reasons to show")
+    ap.add_argument("--min-trials", type=int, default=0, help="Require at least this many trials (sanity)")
+    args = ap.parse_args()
+
+    if not os.path.exists(args.summary):
+        print(f"Not found: {args.summary}")
+        return 2
+
+    summary = _load_json(args.summary)
+    results = summary.get("results", []) or []
+    if not isinstance(results, list):
+        print("Invalid sweep_summary.json: 'results' is not a list")
+        return 2
+
+    if len(results) < args.min_trials:
+        print(f"Too few trials: {len(results)} < {args.min_trials}")
+        return 2
+
+    rows = [_parse_trial(t) for t in results if isinstance(t, dict)]
+    if not rows:
+        print("No trial rows found.")
+        return 2
+
+    # Global aggregation
+    c_all = Counter()
+    for r in rows:
+        for reason in r.abort_reasons:
+            c_all[_norm_reason(reason)] += 1
+    _print_top(c_all, "Abort reasons (all)", limit=args.limit)
+
+    # Split by gravity ON/OFF
+    c_on = Counter()
+    c_off = Counter()
+    for r in rows:
+        target = c_on if r.grav_on else c_off
+        for reason in r.abort_reasons:
+            target[_norm_reason(reason)] += 1
+    _print_top(c_off, "Abort reasons (gravity OFF)", limit=args.limit)
+    _print_top(c_on, "Abort reasons (gravity ON)", limit=args.limit)
+
+    # Show best few configs by stability (same ordering as sweep best_key)
+    def best_key(r: TrialRow) -> Tuple[float, float, float, float]:
+        # success_rate desc, survival_min desc, score_mean desc, tilt not available here
+        return (r.success_rate, r.survival_min, r.score_mean, 0.0)
+
+    rows_sorted = sorted(rows, key=best_key, reverse=True)
+    print("\n== Top trials by (success_rate, survival_min, score_mean) ==")
+    for r in rows_sorted[: min(10, len(rows_sorted))]:
+        grav = "ON" if r.grav_on else "OFF"
+        gs = f"{r.grav_scale:.2f}" if r.grav_scale is not None else "-"
+        wu = f"{r.warmup_seconds:.2f}" if r.warmup_seconds is not None else "-"
+        print(
+            f"idx={r.idx:3d}  {r.status:10s}  sr={r.success_rate:.2f}  "
+            f"surv_min={r.survival_min:6.3f}  surv_mean={r.survival_mean:6.3f}  "
+            f"score_mean={r.score_mean:8.3f}  grav={grav}  gscale={gs}  warmup={wu}"
+        )
+
+    # If success_rate is zero, help user decide what to change next
+    if max(r.success_rate for r in rows) <= 0.0:
+        top_reason = c_all.most_common(1)[0][0] if c_all else "UNKNOWN"
+        print("\n== Hint ==")
+        print("All trials have success_rate=0.0 (no DONE).")
+        print(f"Most common abort reason (normalized): {top_reason}")
+        print("Next steps usually help:")
+        print("  - Increase warmup_seconds (e.g., 1.0 -> 2.0) and/or settle_steps")
+        print("  - If tilt-related, try higher kd in hold, or slightly lower kp, or gravity_scale sweep")
+        print("  - If base_z low, check initial pose / BASE_HEIGHT / stance width")
+
+    return 0
+
+
+if __name__ == "__main__":
+    raise SystemExit(main())
```

使い方（Docker内）:

```bash
cd /workspace/hunter
python3 scripts/analyze_abort_reasons.py --summary runs/sweep_summary.json --limit 12
```

（もし sweep_summary.json がまだ小さいなら）:

```bash
python3 scripts/analyze_abort_reasons.py --summary runs/sweep_summary.json --min-trials 3
```

これで、**gravity ON/OFF で abort reason がどう変わるか**と、**次にどのパラメータを触るべきか**がすぐ見えるようになります。


```diff
--- a/scripts/compare_runs.py
+++ b/scripts/compare_runs.py
@@ -1,18 +1,19 @@
 #!/usr/bin/env python3
 from __future__ import annotations
 
 import argparse
 import json
 import os
 from dataclasses import dataclass
 from datetime import datetime
-from typing import Any, Dict, List, Optional, Tuple
+from typing import Any, Dict, List, Optional, Tuple
 
 def _import_compute_score(repo_root: str):
     """
     Import src/ext_metrics.py without requiring installation.
     """
@@
 @dataclass
 class RunSummary:
     path: str
     mtime: float
     status: str
     score: Optional[float]
+    grav_on: Optional[bool]
+    grav_scale: Optional[float]
     survival_time: Optional[float]
     tilt_max_abs: Optional[float]
     base_z_min: Optional[float]
     energy_abs_tau_dq: Optional[float]
     foot_slip_left: Optional[float]
@@
 def _safe_get(d: Dict[str, Any], keys: List[str], default=None):
     cur: Any = d
     for k in keys:
         if not isinstance(cur, dict) or k not in cur:
             return default
         cur = cur[k]
     return cur
 
@@
 def _summarize_run(path: str, compute_score_fn) -> Optional[RunSummary]:
     try:
         payload = _load_json(path)
     except Exception:
         return None
 
     result = payload.get("result", {})
     metrics = result.get("metrics", {}) if isinstance(result, dict) else {}
     abort = result.get("abort", None) if isinstance(result, dict) else None
+    meta = payload.get("meta", {}) if isinstance(payload, dict) else {}
 
     st = _safe_get(metrics, ["survival_time"])
     tilt = _safe_get(metrics, ["tilt_max_abs"])
     zmin = _safe_get(metrics, ["base_z_min"])
     energy = _safe_get(metrics, ["energy_abs_tau_dq"])
@@
     score = None
     try:
         score = float(compute_score_fn(metrics if isinstance(metrics, dict) else {}, str(result.get("status", "UNKNOWN"))))
     except Exception:
         score = None
 
+    grav_on = _safe_get(meta, ["controller", "torque", "use_gravity_comp"], default=None)
+    grav_scale = _safe_get(meta, ["controller", "torque", "gravity_scale"], default=None)
+    try:
+        grav_on = bool(grav_on) if grav_on is not None else None
+    except Exception:
+        grav_on = None
+    try:
+        grav_scale = float(grav_scale) if grav_scale is not None else None
+    except Exception:
+        grav_scale = None
+
     return RunSummary(
         path=path,
         mtime=os.path.getmtime(path),
         status=str(result.get("status", "UNKNOWN")) if isinstance(result, dict) else "UNKNOWN",
         score=score,
+        grav_on=grav_on,
+        grav_scale=grav_scale,
         survival_time=float(st) if st is not None else None,
         tilt_max_abs=float(tilt) if tilt is not None else None,
         base_z_min=float(zmin) if zmin is not None else None,
         energy_abs_tau_dq=float(energy) if energy is not None else None,
@@
 def _fmt(x: Optional[float], digits: int = 3) -> str:
     if x is None:
         return "-"
     return f"{x:.{digits}f}"
 
+def _fmt_grav_on(x: Optional[bool]) -> str:
+    if x is None:
+        return "-"
+    return "ON" if x else "OFF"
+
 def _fmt_time(ts: float) -> str:
     return datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S")
 
@@
 def _print_table(rows: List[RunSummary], limit: int):
     rows = rows[:limit]
     headers = [
         "rank",
         "time",
         "status",
         "score",
+        "grav",
+        "g_scale",
         "survival_s",
         "tilt_max(rad)",
         "base_z_min",
         "energy",
         "slip_L",
@@
     data = []
     for i, r in enumerate(rows, 1):
         data.append([
             str(i),
             _fmt_time(r.mtime),
             r.status,
             _fmt(r.score, 3),
+            _fmt_grav_on(r.grav_on),
+            _fmt(r.grav_scale, 3),
             _fmt(r.survival_time, 3),
             _fmt(r.tilt_max_abs, 3),
             _fmt(r.base_z_min, 3),
             _fmt(r.energy_abs_tau_dq, 3),
             _fmt(r.foot_slip_left, 3),
@@
 def main() -> int:
     ap = argparse.ArgumentParser()
     ap.add_argument("--log-dir", default="runs")
     ap.add_argument("--prefix", default="")
     ap.add_argument("--limit", type=int, default=10)
     ap.add_argument(
         "--sort-by",
         default="mtime",
         choices=["mtime", "survival", "score", "tilt", "energy"],
         help="Sort criterion (default: newest first)",
     )
+    ap.add_argument(
+        "--grav",
+        default="any",
+        choices=["any", "on", "off"],
+        help="Filter by gravity compensation flag stored in meta.controller.torque.use_gravity_comp",
+    )
+    ap.add_argument("--gscale-min", type=float, default=None, help="Filter: gravity_scale >= this")
+    ap.add_argument("--gscale-max", type=float, default=None, help="Filter: gravity_scale <= this")
     args = ap.parse_args()
 
     repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
     compute_score_fn = _import_compute_score(repo_root)
@@
     runs: List[RunSummary] = []
     for p in paths:
         s = _summarize_run(p, compute_score_fn)
         if s is not None:
             runs.append(s)
 
     if not runs:
         print("Logs were found but none could be parsed.")
         return 2
+
+    # Filters
+    if args.grav != "any":
+        want = True if args.grav == "on" else False
+        runs = [r for r in runs if r.grav_on is not None and r.grav_on == want]
+
+    if args.gscale_min is not None:
+        runs = [r for r in runs if r.grav_scale is not None and r.grav_scale >= args.gscale_min]
+    if args.gscale_max is not None:
+        runs = [r for r in runs if r.grav_scale is not None and r.grav_scale <= args.gscale_max]
+
+    if not runs:
+        print("No runs remain after filtering.")
+        return 0
 
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
     return 0
```

### 使い方例（重力補償 ON/OFF を score で比較しやすくする）

```bash
cd /workspace/hunter

# 全体を score順に（grav列/scale列つき）
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --limit 15

# 重力補償 OFF だけ
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --grav off --limit 10

# 重力補償 ON だけ
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --grav on --limit 10

# gravity_scale を範囲で絞る（例: 0.8〜1.2）
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --grav on --gscale-min 0.8 --gscale-max 1.2 --limit 10
```

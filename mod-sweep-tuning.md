```diff
--- a/scripts/sweep_tuning.py
+++ b/scripts/sweep_tuning.py
@@ -1,5 +1,6 @@
 #!/usr/bin/env python3
 from __future__ import annotations
+
 
 import argparse
 import itertools
 import json
@@ -96,6 +97,28 @@
 def _score(tr: TrialResult) -> float:
     return float(tr.score_mean)
 
+
+def _success_rate(tr: TrialResult) -> float:
+    if tr.repeats <= 0:
+        return 0.0
+    return float(tr.success_count) / float(tr.repeats)
+
+
+def _best_key(tr: TrialResult) -> tuple:
+    """
+    Selection policy (practical/stable):
+      1) success_rate (DONE fraction)  [higher is better]
+      2) survival_min (worst-case)     [higher is better]
+      3) score_mean (average score)    [higher is better]
+      4) tilt_mean (tie-breaker)       [lower is better]
+    """
+    sr = _success_rate(tr)
+    sm = float(tr.survival_min)
+    sc = float(tr.score_mean)
+    tilt = float(tr.tilt_mean) if tr.tilt_mean is not None else 1e9
+    # For max() comparisons, invert tilt so "smaller tilt" becomes "larger is better"
+    return (sr, sm, sc, -tilt)
+
 
 def _extract_single(log_path: str, compute_score_fn) -> Tuple[str, Dict[str, Any], float, Optional[float], Optional[float], Optional[str]]:
     payload = _load_json(log_path)
     result = payload.get("result", {})
     metrics = result.get("metrics", {}) if isinstance(result, dict) else {}
@@ -256,7 +279,7 @@
         tr = TrialResult(
             idx=i,
             params=p,
             repeats=repeats,
             success_count=success_count,
             status=f"{'DONE' if success_count==repeats else 'ABORT'}@{success_count}/{repeats}",
             survival_mean=float(survival_mean),
             survival_min=float(survival_min),
             score_mean=float(score_mean),
             tilt_mean=float(tilt_mean) if tilt_mean is not None else None,
             energy_mean=float(energy_mean) if energy_mean is not None else None,
             abort_reasons=abort_reasons,
             log_paths=log_paths,
         )
 
         results.append(tr)
         print(
-            f"  {tr.status}  survival_mean={tr.survival_mean:.3f}  survival_min={tr.survival_min:.3f}  "
-            f"score_mean={tr.score_mean:.3f}  tilt_mean={tr.tilt_mean if tr.tilt_mean is not None else '-'}"
+            f"  {tr.status}  success_rate={_success_rate(tr):.2f}  "
+            f"survival_mean={tr.survival_mean:.3f}  survival_min={tr.survival_min:.3f}  "
+            f"score_mean={tr.score_mean:.3f}  tilt_mean={tr.tilt_mean if tr.tilt_mean is not None else '-'}"
         )
 
-        if best is None or _score(tr) > _score(best):
+        # Best selection: success_rate -> survival_min -> score_mean -> tilt_mean
+        if best is None or _best_key(tr) > _best_key(best):
             best = tr
 
     if best is None:
         print("No successful trials.")
         return 2
@@ -320,13 +343,17 @@
     print("\n== Best ==")
     print("  params:", best.params)
     print("  status:", best.status)
+    print(f"  success_rate: {_success_rate(best):.2f} ({best.success_count}/{best.repeats})")
     print("  survival_mean:", best.survival_mean)
     print("  survival_min:", best.survival_min)
     print("  score_mean:", best.score_mean)
     print("  tilt_mean:", best.tilt_mean)
     print("  logs:", best.log_paths[-3:] if best.log_paths else [])
     print("  wrote:", args.best_out)
 
     # Save a compact summary JSON
     summary_path = os.path.join(repo_root, args.log_dir, "sweep_summary.json")
     with open(summary_path, "w", encoding="utf-8") as f:
         json.dump(
             {
                 "best": {
                     "params": best.params,
                     "status": best.status,
                     "repeats": best.repeats,
                     "success_count": best.success_count,
+                    "success_rate": _success_rate(best),
                     "survival_mean": best.survival_mean,
                     "survival_min": best.survival_min,
                     "score_mean": best.score_mean,
                     "tilt_mean": best.tilt_mean,
                     "log_paths": best.log_paths,
                 },
                 "results": [
                     {
                         "idx": r.idx,
                         "params": r.params,
                         "status": r.status,
                         "repeats": r.repeats,
                         "success_count": r.success_count,
+                        "success_rate": _success_rate(r),
                         "survival_mean": r.survival_mean,
                         "survival_min": r.survival_min,
                         "score_mean": r.score_mean,
                         "tilt_mean": r.tilt_mean,
                         "energy_mean": r.energy_mean,
                         "abort_reasons": r.abort_reasons[:10],
                         "log_paths": r.log_paths[-3:],
                     }
                     for r in results
                 ],
             },
             f,
             ensure_ascii=False,
             indent=2,
         )
```

### 期待される挙動（確認ポイント）

* `--repeats 2` 以上のとき、best の選定が **DONE率（success_rate）優先**になります
* 同じ DONE率なら **survival_min**（最悪値）が大きいものを採用
* さらに同じなら **score_mean**、最後に **tilt_mean** が小さい方を採用
* `runs/sweep_summary.json` に `success_rate` が追加されます

必要なら、`compare_runs.py` 側にも「meta.controller.torque.use_gravity_comp」「meta.controller.torque.gravity_scale」を列表示して、ON/OFF・scaleが表で一目で分かるようにするパッチも出せます。

```diff
--- a/scripts/compare_runs.py
+++ b/scripts/compare_runs.py
@@ -1,17 +1,18 @@
 #!/usr/bin/env python3
 from __future__ import annotations
 
 import argparse
 import json
 import os
 from dataclasses import dataclass
 from datetime import datetime
 from typing import Any, Dict, List, Optional, Tuple
 
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
     foot_slip_right: Optional[float]
     abort_reason: Optional[str]
@@
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
+    meta = payload.get("meta", {}) if isinstance(payload, dict) else {}
 
     st = _safe_get(metrics, ["survival_time"])
     tilt = _safe_get(metrics, ["tilt_max_abs"])
     zmin = _safe_get(metrics, ["base_z_min"])
     energy = _safe_get(metrics, ["energy_abs_tau_dq"])
     slip_l = _safe_get(metrics, ["foot_slip_left"])
     slip_r = _safe_get(metrics, ["foot_slip_right"])
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
         foot_slip_left=float(slip_l) if slip_l is not None else None,
         foot_slip_right=float(slip_r) if slip_r is not None else None,
         abort_reason=str(reason) if reason is not None else None,
     )
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
         "slip_R",
         "abort_reason",
         "file",
     ]
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
             _fmt(r.foot_slip_right, 3),
             (r.abort_reason or "-")[:80],
             os.path.basename(r.path),
         ])
```

使い方（Docker内）はそのままで、表示に `grav`（ON/OFF）と `g_scale`（gravity_scale）が増えます：

```bash
cd /workspace/hunter
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --limit 10
```

以下に **(A) score の共通化（`ext_metrics.py`）**、**(B) `compare_runs.py` に score 列追加**、**(C) `sweep_tuning.py` に `--repeats`（複数回評価＋集計）追加** をまとめた **unified diff** を提示します。

---

## 1) `src/ext_metrics.py`：`compute_score()` 追加（共通スコア関数）

```diff
--- a/src/ext_metrics.py
+++ b/src/ext_metrics.py
@@ -1,8 +1,9 @@
 from __future__ import annotations
 from typing import Dict, Any, List
 import math
 
 
 def _energy_step(tau: Dict[str, float], dq: Dict[str, float], dt: float) -> float:
@@ -13,6 +14,45 @@
     return e
 
 
+def compute_score(metrics: Dict[str, Any], status: str) -> float:
+    """
+    Common scoring function used by scripts (compare/sweep).
+
+    Design:
+      - Primary: maximize survival_time
+      - Strong bonus if DONE
+      - Small penalties for tilt and energy (energy is approximate unless torque commands logged)
+    """
+    survival = float(metrics.get("survival_time", 0.0) or 0.0)
+    tilt = metrics.get("tilt_max_abs", None)
+    energy = metrics.get("energy_abs_tau_dq", None)
+
+    score = survival
+    if str(status).upper() == "DONE":
+        score += 100.0
+
+    # Penalize large tilt (rad). Keep weight small to avoid fighting primary objective early.
+    if tilt is not None:
+        try:
+            score -= 0.5 * float(tilt)
+        except Exception:
+            pass
+
+    # Penalize energy lightly (scale depends on dt/logging; keep tiny)
+    if energy is not None:
+        try:
+            score -= 1e-3 * float(energy)
+        except Exception:
+            pass
+
+    return float(score)
+
+
 def compute_metrics(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
     """
     samples: list of dicts produced by ext_runner (one per control update)
     """
```

---

## 2) `scripts/compare_runs.py`：score列を追加（`ext_metrics.compute_score()` を使用）

```diff
--- a/scripts/compare_runs.py
+++ b/scripts/compare_runs.py
@@ -1,17 +1,30 @@
 #!/usr/bin/env python3
 from __future__ import annotations
 
 import argparse
 import json
 import os
 from dataclasses import dataclass
 from datetime import datetime
 from typing import Any, Dict, List, Optional, Tuple
 
+def _import_compute_score(repo_root: str):
+    """
+    Import src/ext_metrics.py without requiring installation.
+    """
+    import sys
+    src_dir = os.path.join(repo_root, "src")
+    if src_dir not in sys.path:
+        sys.path.insert(0, src_dir)
+    from ext_metrics import compute_score  # type: ignore
+    return compute_score
+
 
 @dataclass
 class RunSummary:
     path: str
     mtime: float
     status: str
+    score: Optional[float]
     survival_time: Optional[float]
     tilt_max_abs: Optional[float]
     base_z_min: Optional[float]
     energy_abs_tau_dq: Optional[float]
@@ -34,7 +47,7 @@
     return cur
 
 
 def _load_json(path: str) -> Dict[str, Any]:
@@ -44,7 +57,7 @@
         return json.load(f)
 
 
-def _summarize_run(path: str) -> Optional[RunSummary]:
+def _summarize_run(path: str, compute_score_fn) -> Optional[RunSummary]:
     try:
         payload = _load_json(path)
     except Exception:
         return None
@@ -59,6 +72,7 @@
     st = _safe_get(metrics, ["survival_time"])
     tilt = _safe_get(metrics, ["tilt_max_abs"])
     zmin = _safe_get(metrics, ["base_z_min"])
     energy = _safe_get(metrics, ["energy_abs_tau_dq"])
@@ -71,12 +85,20 @@
     reason = None
     if isinstance(abort, dict):
         reason = abort.get("reason")
 
+    score = None
+    try:
+        score = float(compute_score_fn(metrics if isinstance(metrics, dict) else {}, str(result.get("status", "UNKNOWN"))))
+    except Exception:
+        score = None
+
     return RunSummary(
         path=path,
         mtime=os.path.getmtime(path),
         status=str(result.get("status", "UNKNOWN")) if isinstance(result, dict) else "UNKNOWN",
+        score=score,
         survival_time=float(st) if st is not None else None,
         tilt_max_abs=float(tilt) if tilt is not None else None,
         base_z_min=float(zmin) if zmin is not None else None,
         energy_abs_tau_dq=float(energy) if energy is not None else None,
@@ -118,6 +140,7 @@
     headers = [
         "rank",
         "time",
         "status",
+        "score",
         "survival_s",
         "tilt_max(rad)",
         "base_z_min",
         "energy",
@@ -132,6 +155,7 @@
         data.append([
             str(i),
             _fmt_time(r.mtime),
             r.status,
+            _fmt(r.score, 3),
             _fmt(r.survival_time, 3),
             _fmt(r.tilt_max_abs, 3),
             _fmt(r.base_z_min, 3),
             _fmt(r.energy_abs_tau_dq, 3),
@@ -184,6 +208,9 @@
     args = ap.parse_args()
 
+    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
+    compute_score_fn = _import_compute_score(repo_root)
+
     paths = _find_logs(args.log_dir, args.prefix)
     if not paths:
         print(f"No logs found in '{args.log_dir}' with prefix '{args.prefix}'.")
@@ -192,10 +219,10 @@
 
     runs: List[RunSummary] = []
     for p in paths:
-        s = _summarize_run(p)
+        s = _summarize_run(p, compute_score_fn)
         if s is not None:
             runs.append(s)
 
     if not runs:
         print("Logs were found but none could be parsed.")
@@ -208,6 +235,10 @@
     if args.sort_by == "mtime":
         runs.sort(key=lambda r: r.mtime, reverse=True)
     elif args.sort_by == "survival":
         runs.sort(key=lambda r: (r.survival_time is None, -(r.survival_time or 0.0), -r.mtime))
+    elif args.sort_by == "score":
+        runs.sort(key=lambda r: (r.score is None, -(r.score or 0.0), -r.mtime))
     elif args.sort_by == "tilt":
         runs.sort(key=lambda r: (r.tilt_max_abs is None, (r.tilt_max_abs or 0.0), -r.mtime))
     elif args.sort_by == "energy":
         runs.sort(key=lambda r: (r.energy_abs_tau_dq is None, (r.energy_abs_tau_dq or 0.0), -r.mtime))
@@ -215,6 +246,14 @@
     _print_table(runs, args.limit)
 
     best = runs[0]
     print("\nBest (by sort):", os.path.join(args.log_dir, os.path.basename(best.path)))
```

> 補足：`--sort-by` に `score` を追加したい場合は、choices にも入れてください（次のdiffで入れています）。

```diff
--- a/scripts/compare_runs.py
+++ b/scripts/compare_runs.py
@@ -171,7 +171,7 @@
     ap.add_argument(
         "--sort-by",
         default="mtime",
-        choices=["mtime", "survival", "tilt", "energy"],
+        choices=["mtime", "survival", "score", "tilt", "energy"],
         help="Sort criterion (default: newest first)",
     )
```

---

## 3) `scripts/sweep_tuning.py`：`--repeats` 追加（複数回実行して平均/成功率で評価）

* 同一パラメータを `repeats` 回回す
* 各回のログから `metrics` と `score` を読み
* `score_mean / survival_mean / success_rate` を集計して比較
* best.yaml と sweep_summary.json も集計結果を保存

```diff
--- a/scripts/sweep_tuning.py
+++ b/scripts/sweep_tuning.py
@@ -1,15 +1,16 @@
 #!/usr/bin/env python3
 from __future__ import annotations
 
 import argparse
 import itertools
 import json
 import os
 import random
 import subprocess
 import sys
 import time
 from dataclasses import dataclass
 from typing import Any, Dict, List, Optional, Tuple
 
+def _import_compute_score(repo_root: str):
+    import sys as _sys
+    src_dir = os.path.join(repo_root, "src")
+    if src_dir not in _sys.path:
+        _sys.path.insert(0, src_dir)
+    from ext_metrics import compute_score  # type: ignore
+    return compute_score
+
 
 def _load_json(path: str) -> Dict[str, Any]:
     with open(path, "r", encoding="utf-8") as f:
         return json.load(f)
@@ -84,37 +95,46 @@
 class TrialResult:
     idx: int
     params: Dict[str, Any]
-    log_path: Optional[str]
-    status: str
-    survival_time: float
-    tilt_max_abs: Optional[float]
-    base_z_min: Optional[float]
-    energy: Optional[float]
-    abort_reason: Optional[str]
+    # aggregated across repeats
+    repeats: int
+    success_count: int
+    status: str  # e.g. "DONE@2/3" or "ABORT@0/3"
+    survival_mean: float
+    survival_min: float
+    score_mean: float
+    tilt_mean: Optional[float]
+    energy_mean: Optional[float]
+    abort_reasons: List[str]
+    log_paths: List[str]
 
 
-def _score(tr: TrialResult) -> float:
-    """
-    Primary objective: maximize survival_time.
-    Add big bonus if DONE. Small penalty for tilt.
-    """
-    s = tr.survival_time
-    if tr.status == "DONE":
-        s += 100.0
-    if tr.tilt_max_abs is not None:
-        s -= 0.5 * tr.tilt_max_abs
-    return s
+def _score(tr: TrialResult) -> float:
+    return float(tr.score_mean)
 
 
-def _extract_trial(log_path: str, idx: int, params: Dict[str, Any]) -> TrialResult:
+def _extract_single(log_path: str, compute_score_fn) -> Tuple[str, Dict[str, Any], float, Optional[float], Optional[float], Optional[str]]:
     payload = _load_json(log_path)
     result = payload.get("result", {})
     metrics = result.get("metrics", {}) if isinstance(result, dict) else {}
     abort = result.get("abort", {}) if isinstance(result, dict) else {}
 
     status = str(result.get("status", "UNKNOWN"))
     survival = float(metrics.get("survival_time", 0.0) or 0.0)
     tilt = metrics.get("tilt_max_abs", None)
     zmin = metrics.get("base_z_min", None)
     energy = metrics.get("energy_abs_tau_dq", None)
     reason = abort.get("reason") if isinstance(abort, dict) else None
-
-    return TrialResult(
-        idx=idx,
-        params=params,
-        log_path=log_path,
-        status=status,
-        survival_time=survival,
-        tilt_max_abs=float(tilt) if tilt is not None else None,
-        base_z_min=float(zmin) if zmin is not None else None,
-        energy=float(energy) if energy is not None else None,
-        abort_reason=str(reason) if reason is not None else None,
-    )
+    score = float(compute_score_fn(metrics if isinstance(metrics, dict) else {}, status))
+    return (
+        status,
+        metrics if isinstance(metrics, dict) else {},
+        survival,
+        float(tilt) if tilt is not None else None,
+        float(energy) if energy is not None else None,
+        str(reason) if reason is not None else None,
+    )
 
 
 def _mk_grid(values: List[float]) -> List[float]:
@@ -130,6 +150,7 @@
     ap.add_argument("--best-out", default="config/agent_tuning_best.yaml", help="Where to write best YAML")
     ap.add_argument("--log-dir", default="runs", help="Run logs directory")
     ap.add_argument("--prefix", default="standing_pd_ext", help="Log prefix")
     ap.add_argument("--trials", type=int, default=24, help="Number of trials (random mode)")
+    ap.add_argument("--repeats", type=int, default=1, help="Repeat each parameter set N times and aggregate")
     ap.add_argument("--mode", choices=["grid", "random"], default="grid", help="Sweep mode")
     ap.add_argument("--seed", type=int, default=0, help="Random seed (random mode)")
     ap.add_argument("--duration", type=float, default=None, help="Override runner.seconds (optional)")
     ap.add_argument("--no-gui", action="store_true", help="Force --no-gui for main_simulation.py")
     ap.add_argument("--dry-run", action="store_true", help="Print planned trials but do not run")
@@ -146,6 +167,9 @@
     repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
     src_dir = os.path.join(repo_root, "src")
 
+    compute_score_fn = _import_compute_score(repo_root)
+
     base_cfg = _load_yaml(os.path.join(repo_root, args.config))
     runner_cfg = (base_cfg.get("runner") or {})
     ctrl_cfg = (base_cfg.get("controller") or {})
@@ -206,12 +230,16 @@
     best: Optional[TrialResult] = None
     results: List[TrialResult] = []
 
     for i, p in enumerate(planned_params, 1):
@@ -237,6 +265,15 @@
         cfg["runner"] = cfg_runner
         cfg["controller"] = cfg_ctrl
 
+        # Optional: set runner.seed (even if not used yet, harmless)
+        base_seed = int((runner_cfg.get("seed", 0) or 0))
+        cfg_runner["seed"] = base_seed
+
         # Save into the config path used by main_simulation.py
         cfg_path = os.path.join(repo_root, args.config)
         _dump_yaml(cfg_path, cfg)
 
@@ -253,10 +290,6 @@
             cmd.append("--no-gui")
 
-        # capture latest log before run to detect new one
-        before = _find_latest_log(os.path.join(repo_root, args.log_dir), args.prefix)
-        before_mtime = os.path.getmtime(before) if before else 0.0
-
-        print(f"\n== Trial {i}/{len(planned_params)} == {p}")
-        rc = _run(cmd, cwd=src_dir)
-        if rc != 0:
-            tr = TrialResult(
-                idx=i,
-                params=p,
-                log_path=None,
-                status=f"EXIT_{rc}",
-                survival_time=0.0,
-                tilt_max_abs=None,
-                base_z_min=None,
-                energy=None,
-                abort_reason=f"process exited {rc}",
-            )
-            results.append(tr)
-            continue
-
-        # find the newest log produced after this run
-        latest = _find_latest_log(os.path.join(repo_root, args.log_dir), args.prefix)
-        if latest is None:
-            tr = TrialResult(
-                idx=i,
-                params=p,
-                log_path=None,
-                status="NO_LOG",
-                survival_time=0.0,
-                tilt_max_abs=None,
-                base_z_min=None,
-                energy=None,
-                abort_reason="no log found",
-            )
-            results.append(tr)
-            continue
-
-        latest_mtime = os.path.getmtime(latest)
-        if latest_mtime <= before_mtime:
-            # log wasn't updated; still parse it but mark
-            tr = _extract_trial(latest, i, p)
-            tr.status = tr.status + "_STALELOG"
-        else:
-            tr = _extract_trial(latest, i, p)
-
-        results.append(tr)
-
-        sc = _score(tr)
-        print(
-            f"  status={tr.status} survival={tr.survival_time:.3f} "
-            f"tilt={tr.tilt_max_abs if tr.tilt_max_abs is not None else '-'} "
-            f"score={sc:.3f} log={os.path.basename(latest) if latest else '-'}"
-        )
-        if best is None or sc > _score(best):
-            best = tr
+        repeats = max(1, int(args.repeats))
+        print(f"\n== Trial {i}/{len(planned_params)} == {p} (repeats={repeats})")
+
+        statuses: List[str] = []
+        survivals: List[float] = []
+        scores: List[float] = []
+        tilts: List[float] = []
+        energies: List[float] = []
+        abort_reasons: List[str] = []
+        log_paths: List[str] = []
+
+        for r in range(repeats):
+            # capture latest log before run to detect new one
+            before = _find_latest_log(os.path.join(repo_root, args.log_dir), args.prefix)
+            before_mtime = os.path.getmtime(before) if before else 0.0
+
+            # update seed per repeat (if main uses it, enables reproducibility)
+            cfg = _load_yaml(cfg_path)
+            cfg_runner2 = (cfg.get("runner") or {})
+            cfg_runner2["seed"] = base_seed + r
+            cfg["runner"] = cfg_runner2
+            _dump_yaml(cfg_path, cfg)
+
+            rc = _run(cmd, cwd=src_dir)
+            if rc != 0:
+                statuses.append(f"EXIT_{rc}")
+                survivals.append(0.0)
+                scores.append(0.0)
+                abort_reasons.append(f"process exited {rc}")
+                continue
+
+            latest = _find_latest_log(os.path.join(repo_root, args.log_dir), args.prefix)
+            if latest is None:
+                statuses.append("NO_LOG")
+                survivals.append(0.0)
+                scores.append(0.0)
+                abort_reasons.append("no log found")
+                continue
+
+            latest_mtime = os.path.getmtime(latest)
+            status, metrics, survival, tilt, energy, reason = _extract_single(latest, compute_score_fn)
+            if latest_mtime <= before_mtime:
+                status = status + "_STALELOG"
+
+            statuses.append(status)
+            survivals.append(float(survival))
+            scores.append(float(compute_score_fn(metrics, status.replace("_STALELOG",""))))
+            if tilt is not None:
+                tilts.append(float(tilt))
+            if energy is not None:
+                energies.append(float(energy))
+            if reason is not None:
+                abort_reasons.append(str(reason))
+            log_paths.append(latest)
+
+        success_count = sum(1 for s in statuses if str(s).upper().startswith("DONE"))
+        survival_mean = sum(survivals) / len(survivals) if survivals else 0.0
+        survival_min = min(survivals) if survivals else 0.0
+        score_mean = sum(scores) / len(scores) if scores else 0.0
+        tilt_mean = (sum(tilts) / len(tilts)) if tilts else None
+        energy_mean = (sum(energies) / len(energies)) if energies else None
+
+        tr = TrialResult(
+            idx=i,
+            params=p,
+            repeats=repeats,
+            success_count=success_count,
+            status=f"{'DONE' if success_count==repeats else 'ABORT'}@{success_count}/{repeats}",
+            survival_mean=float(survival_mean),
+            survival_min=float(survival_min),
+            score_mean=float(score_mean),
+            tilt_mean=float(tilt_mean) if tilt_mean is not None else None,
+            energy_mean=float(energy_mean) if energy_mean is not None else None,
+            abort_reasons=abort_reasons,
+            log_paths=log_paths,
+        )
+
+        results.append(tr)
+        print(
+            f"  {tr.status}  survival_mean={tr.survival_mean:.3f}  survival_min={tr.survival_min:.3f}  "
+            f"score_mean={tr.score_mean:.3f}  tilt_mean={tr.tilt_mean if tr.tilt_mean is not None else '-'}"
+        )
+
+        if best is None or _score(tr) > _score(best):
+            best = tr
 
     if best is None:
         print("No successful trials.")
         return 2
@@ -299,6 +394,7 @@
     best_cfg = _load_yaml(os.path.join(repo_root, args.config))
     best_cfg_runner = best_cfg.get("runner") or {}
     best_cfg_ctrl = best_cfg.get("controller") or {}
     best_cfg_runner["control_dt"] = float(best.params["control_dt"])
     best_cfg_runner["settle_steps"] = int(best.params["settle_steps"])
@@ -329,25 +425,33 @@
 
     print("\n== Best ==")
     print("  params:", best.params)
     print("  status:", best.status)
-    print("  survival:", best.survival_time)
-    print("  tilt_max:", best.tilt_max_abs)
-    print("  log:", best.log_path)
+    print("  survival_mean:", best.survival_mean)
+    print("  survival_min:", best.survival_min)
+    print("  score_mean:", best.score_mean)
+    print("  tilt_mean:", best.tilt_mean)
+    print("  logs:", best.log_paths[-3:] if best.log_paths else [])
     print("  wrote:", args.best_out)
 
     # Save a compact summary JSON
     summary_path = os.path.join(repo_root, args.log_dir, "sweep_summary.json")
     with open(summary_path, "w", encoding="utf-8") as f:
         json.dump(
             {
                 "best": {
                     "params": best.params,
                     "status": best.status,
-                    "survival_time": best.survival_time,
-                    "tilt_max_abs": best.tilt_max_abs,
-                    "log_path": best.log_path,
+                    "repeats": best.repeats,
+                    "success_count": best.success_count,
+                    "survival_mean": best.survival_mean,
+                    "survival_min": best.survival_min,
+                    "score_mean": best.score_mean,
+                    "tilt_mean": best.tilt_mean,
+                    "log_paths": best.log_paths,
                 },
                 "results": [
                     {
                         "idx": r.idx,
                         "params": r.params,
                         "status": r.status,
-                        "survival_time": r.survival_time,
-                        "tilt_max_abs": r.tilt_max_abs,
-                        "base_z_min": r.base_z_min,
-                        "energy": r.energy,
-                        "abort_reason": r.abort_reason,
-                        "log_path": r.log_path,
+                        "repeats": r.repeats,
+                        "success_count": r.success_count,
+                        "survival_mean": r.survival_mean,
+                        "survival_min": r.survival_min,
+                        "score_mean": r.score_mean,
+                        "tilt_mean": r.tilt_mean,
+                        "energy_mean": r.energy_mean,
+                        "abort_reasons": r.abort_reasons[:10],
+                        "log_paths": r.log_paths[-3:],
                     }
                     for r in results
                 ],
             },
             f,
             ensure_ascii=False,
             indent=2,
         )
```

---

## 使い方（Docker内）

```bash
cd /workspace/hunter

# まず compare で score を見られる
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --limit 10

# sweep を repeats 付きで実行（例：各候補を3回回して平均で評価）
python3 scripts/sweep_tuning.py --mode grid --trials 8 --warmup 0.5,1.0 --repeats 3 --no-gui

# best を survival/score で確認
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --limit 10
```



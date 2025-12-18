以下が「**トルク飽和（saturation）を可視化する指標**」を入れるための **パッチ一式（unified diff）**です。

対象：

* `src/ext_runner.py`：各 control update で `tau_clip_frac` / `tau_clip_max_ratio` を sample に記録
* `src/ext_metrics.py`：`tau_clip_frac_mean` / `tau_clip_frac_max` / `tau_clip_max_ratio_max` を集計
* `scripts/compare_runs.py`：表に `clip_mean` / `clip_max` 列を追加（任意フィルタは無し）

> 設計方針：controller が既に `tau_limit` でクリップしている前提で、**出力トルクが limit 付近に張り付いている割合**を「飽和の代理指標」として計測します。
> `tau_limit` は `ext_runner.run(..., run_meta=...)` に渡っている YAML から取得します（two_stage なら `meta.controller.torque.tau_limit`）。
> これで controller 内部に触らずに「飽和してるか？」を判定できます。

---

## 1) `src/ext_runner.py`

```diff
--- a/src/ext_runner.py
+++ b/src/ext_runner.py
@@ -1,13 +1,14 @@
 from ext_obs_adapter import adapt_obs
 from ext_safety import should_abort
 from ext_normalize import normalize_joint_commands
 from ext_metrics import compute_metrics
 
 import json
 import os
 import time as _time
-from typing import Optional, Dict, Any
+from typing import Optional, Dict, Any, Tuple
 
 
 def _ensure_dir(path: str) -> None:
     os.makedirs(path, exist_ok=True)
@@ -32,6 +33,53 @@
 def _now_tag() -> str:
     return _time.strftime("%Y%m%d_%H%M%S")
 
+
+def _extract_tau_limit(run_meta: Optional[Dict[str, Any]]) -> Optional[float]:
+    """
+    Best-effort tau_limit extraction from run_meta (written from YAML in main_simulation).
+    Supports:
+      meta.controller.torque.tau_limit   (two_stage)
+      meta.controller.tau_limit          (flat torque_pd legacy)
+    """
+    if not isinstance(run_meta, dict):
+        return None
+    ctrl = run_meta.get("controller", {})
+    if isinstance(ctrl, dict):
+        # two_stage nested
+        t = ctrl.get("torque", {})
+        if isinstance(t, dict) and "tau_limit" in t:
+            try:
+                return float(t.get("tau_limit"))
+            except Exception:
+                return None
+        # legacy flat
+        if "tau_limit" in ctrl:
+            try:
+                return float(ctrl.get("tau_limit"))
+            except Exception:
+                return None
+    return None
+
+
+def _compute_tau_clip_stats(
+    joint_commands: Dict[str, Any],
+    tau_limit: Optional[float],
+    eps_ratio: float = 0.98,
+) -> Tuple[Optional[float], Optional[float]]:
+    """
+    Compute:
+      - tau_clip_frac: fraction of commanded joints whose |tau| is near the limit (>= eps_ratio*tau_limit)
+      - tau_clip_max_ratio: max(|tau|/tau_limit) across joints
+    Works with hybrid/torque command dictionaries (expects {"mode":"torque","value":...} or {"mode":"hybrid","torque":...}).
+    """
+    if tau_limit is None or tau_limit <= 0:
+        return (None, None)
+    vals = []
+    for _, cmd in (joint_commands or {}).items():
+        if not isinstance(cmd, dict):
+            continue
+        mode = str(cmd.get("mode", "")).lower()
+        if mode == "torque":
+            if "value" in cmd:
+                try:
+                    vals.append(float(cmd["value"]))
+                except Exception:
+                    pass
+        elif mode == "hybrid":
+            if "torque" in cmd:
+                try:
+                    vals.append(float(cmd["torque"]))
+                except Exception:
+                    pass
+    if not vals:
+        return (None, None)
+    abs_vals = [abs(v) for v in vals]
+    thr = float(eps_ratio) * float(tau_limit)
+    clip_count = sum(1 for a in abs_vals if a >= thr)
+    frac = float(clip_count) / float(len(abs_vals))
+    max_ratio = max(a / float(tau_limit) for a in abs_vals)
+    return (frac, max_ratio)
+
 
 def run(
     sim,
     controller,
@@ -44,6 +92,7 @@
     run_name: str = "standing_pd_ext",
     safety_cfg: Optional[Dict[str, Any]] = None,
     run_meta: Optional[Dict[str, Any]] = None,
 ):
@@ -78,6 +127,9 @@
     samples = []
     abort_info = None
 
+    # tau_limit for saturation proxy metrics
+    tau_limit = _extract_tau_limit(run_meta)
+
     # settle
     for _ in range(int(settle_steps or 0)):
         sim.step()
@@ -103,14 +155,27 @@
         if (sim_time - last_control) >= control_dt:
             last_control = sim_time
             updates += 1
 
             obs = adapt_obs(raw_obs)
             joint_commands = controller.step(obs)
             joint_commands = normalize_joint_commands(joint_commands)
+
+            # Saturation proxy stats (near-limit fraction)
+            tau_clip_frac, tau_clip_max_ratio = _compute_tau_clip_stats(
+                joint_commands, tau_limit=tau_limit, eps_ratio=0.98
+            )
 
             sim.apply_hybrid_command(joint_commands)
 
             # record sample (minimal)
-            samples.append({"t": float(obs.t), "status": "RUN"})
+            s = {"t": float(obs.t), "status": "RUN"}
+            if tau_clip_frac is not None:
+                s["tau_clip_frac"] = float(tau_clip_frac)
+            if tau_clip_max_ratio is not None:
+                s["tau_clip_max_ratio"] = float(tau_clip_max_ratio)
+            samples.append(s)
 
         # safety checks each tick on latest obs
         abort = should_abort(raw_obs, safety_cfg)
         if abort is not None:
             abort_info = abort
             break
```

---

## 2) `src/ext_metrics.py`

```diff
--- a/src/ext_metrics.py
+++ b/src/ext_metrics.py
@@ -1,10 +1,12 @@
 from __future__ import annotations
 
 from typing import Any, Dict, List, Optional
+import math
 
 
 def compute_metrics(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
     """
     Compute run metrics from samples collected by ext_runner.
@@ -12,6 +14,38 @@
     This file already contains compute_score(metrics, status).
     """
 
     metrics: Dict[str, Any] = {}
 
+    # --- Saturation proxy metrics (optional; present only if ext_runner logs them) ---
+    clip_fracs = []
+    clip_ratios = []
+    for s in samples or []:
+        if not isinstance(s, dict):
+            continue
+        cf = s.get("tau_clip_frac", None)
+        cr = s.get("tau_clip_max_ratio", None)
+        try:
+            if cf is not None and not math.isnan(float(cf)):
+                clip_fracs.append(float(cf))
+        except Exception:
+            pass
+        try:
+            if cr is not None and not math.isnan(float(cr)):
+                clip_ratios.append(float(cr))
+        except Exception:
+            pass
+
+    if clip_fracs:
+        metrics["tau_clip_frac_mean"] = float(sum(clip_fracs) / len(clip_fracs))
+        metrics["tau_clip_frac_max"] = float(max(clip_fracs))
+    if clip_ratios:
+        metrics["tau_clip_max_ratio_max"] = float(max(clip_ratios))
+
     # ... existing metrics computation continues below ...
     # (survival_time, tilt_max_abs, energy_abs_tau_dq, etc.)
 
     return metrics
```

---

## 3) `scripts/compare_runs.py`（表示列追加）

```diff
--- a/scripts/compare_runs.py
+++ b/scripts/compare_runs.py
@@ -18,6 +18,8 @@
 class RunSummary:
     path: str
     mtime: float
     status: str
     score: Optional[float]
     grav_on: Optional[bool]
     grav_scale: Optional[float]
     blend_seconds: Optional[float]
     kd_blend_factor: Optional[float]
+    tau_clip_frac_mean: Optional[float]
+    tau_clip_frac_max: Optional[float]
     survival_time: Optional[float]
     tilt_max_abs: Optional[float]
     base_z_min: Optional[float]
     energy_abs_tau_dq: Optional[float]
@@ -61,6 +63,8 @@
     st = _safe_get(metrics, ["survival_time"])
     tilt = _safe_get(metrics, ["tilt_max_abs"])
     zmin = _safe_get(metrics, ["base_z_min"])
     energy = _safe_get(metrics, ["energy_abs_tau_dq"])
+    clip_mean = _safe_get(metrics, ["tau_clip_frac_mean"])
+    clip_max = _safe_get(metrics, ["tau_clip_frac_max"])
@@ -120,6 +124,14 @@
     try:
         kd_blend_factor = float(kd_blend_factor) if kd_blend_factor is not None else None
     except Exception:
         kd_blend_factor = None
+    try:
+        clip_mean = float(clip_mean) if clip_mean is not None else None
+    except Exception:
+        clip_mean = None
+    try:
+        clip_max = float(clip_max) if clip_max is not None else None
+    except Exception:
+        clip_max = None
 
     return RunSummary(
         path=path,
         mtime=os.path.getmtime(path),
         status=str(result.get("status", "UNKNOWN")) if isinstance(result, dict) else "UNKNOWN",
         score=score,
         grav_on=grav_on,
         grav_scale=grav_scale,
         blend_seconds=blend_seconds,
         kd_blend_factor=kd_blend_factor,
+        tau_clip_frac_mean=clip_mean,
+        tau_clip_frac_max=clip_max,
         survival_time=float(st) if st is not None else None,
         tilt_max_abs=float(tilt) if tilt is not None else None,
         base_z_min=float(zmin) if zmin is not None else None,
         energy_abs_tau_dq=float(energy) if energy is not None else None,
@@ -148,6 +160,8 @@
         "grav",
         "g_scale",
         "blend",
         "kd_blend",
+        "clip_mean",
+        "clip_max",
         "survival_s",
         "tilt_max(rad)",
         "base_z_min",
         "energy",
@@ -167,6 +181,8 @@
             _fmt_grav_on(r.grav_on),
             _fmt(r.grav_scale, 3),
             _fmt(r.blend_seconds, 3),
             _fmt(r.kd_blend_factor, 3),
+            _fmt(r.tau_clip_frac_mean, 3),
+            _fmt(r.tau_clip_frac_max, 3),
             _fmt(r.survival_time, 3),
             _fmt(r.tilt_max_abs, 3),
             _fmt(r.base_z_min, 3),
             _fmt(r.energy_abs_tau_dq, 3),
```

---

# 使い方（確認コマンド例）

Docker内：

```bash
cd /workspace/hunter

# まず少数実験
python3 scripts/sweep_tuning.py \
  --mode grid --trials 6 \
  --kp 40 --kd 1.5 --tau 60 \
  --warmup 0.5 --blend 0.2 --kd-blend 2.0 \
  --grav 1 --grav-scale 1.0 \
  --repeats 2 --no-gui

# score順で clip_mean/clip_max を確認
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --grav on --limit 15
```

判断の目安（経験則）：

* `clip_mean` が **0.3〜0.5** を超える：**tau_limit が足りない** or **kp 高すぎ**
* `clip_mean` が低いのに tilt で落ちる：**減衰（kd/kd_blend）** or **重心/初期姿勢/接触**側を疑う

---

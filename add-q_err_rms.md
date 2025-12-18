了解。ここまでの結果（**stable contacts は既にON／position-onlyでも 1秒未満／pos_kd を振ってもほぼ同じ**）だと、次にやるべきはゲイン探索の続行ではなく、

**「position制御が“目標姿勢に寄せられているのか”を数値で確定する」**
これが最短です。

そのために **q_err_rms（関節目標との差のRMS）＋ roll/pitch をサンプルに記録 → metrics集計 → compare表示** を入れます。
これで、

* `q_err_rms` がほぼ下がらない → **position制御が効いていない / 目標姿勢が遠すぎる / 関節対応ミス / gain単位が期待と違う**
* `q_err_rms` は下がるのに tilt → **目標姿勢(q_ref)自体が不安定（直脚・腰高・足幅不足など） or ベース高さ/接触の問題**

を一撃で分岐できます。

以下、**パッチ一式（unified diff）**です。

---

# 1) `src/ext_runner.py`（q_err_rms + roll/pitch を samples に入れる）

```diff
--- a/src/ext_runner.py
+++ b/src/ext_runner.py
@@ -1,13 +1,15 @@
 from ext_obs_adapter import adapt_obs
 from ext_safety import should_abort
 from ext_normalize import normalize_joint_commands
 from ext_metrics import compute_metrics
 
 import json
 import os
 import time as _time
-from typing import Optional, Dict, Any, Tuple
+from typing import Optional, Dict, Any, Tuple
+import math
 
 
 def _ensure_dir(path: str) -> None:
     os.makedirs(path, exist_ok=True)
@@ -33,6 +35,40 @@
 def _now_tag() -> str:
     return _time.strftime("%Y%m%d_%H%M%S")
 
+def _quat_to_rpy(q):
+    """q: (x,y,z,w) -> (roll,pitch,yaw)"""
+    try:
+        x, y, z, w = [float(v) for v in q]
+    except Exception:
+        return (None, None, None)
+    # standard conversion
+    t0 = +2.0 * (w * x + y * z)
+    t1 = +1.0 - 2.0 * (x * x + y * y)
+    roll = math.atan2(t0, t1)
+    t2 = +2.0 * (w * y - z * x)
+    t2 = max(-1.0, min(1.0, t2))
+    pitch = math.asin(t2)
+    t3 = +2.0 * (w * z + x * y)
+    t4 = +1.0 - 2.0 * (y * y + z * z)
+    yaw = math.atan2(t3, t4)
+    return (roll, pitch, yaw)
+
+def _q_err_rms(obs, controller) -> Optional[float]:
+    """
+    RMS joint error to controller.q_ref if available.
+    obs.q is expected to be a length-10 np array in controller joint order.
+    """
+    try:
+        q_ref = getattr(controller, "q_ref", None)
+        q = getattr(obs, "q", None)
+        if q_ref is None or q is None:
+            return None
+        if len(q_ref) != len(q):
+            return None
+        s = 0.0
+        for a, b in zip(q_ref, q):
+            d = float(a) - float(b)
+            s += d * d
+        return math.sqrt(s / max(1, len(q_ref)))
+    except Exception:
+        return None
 
 def _extract_tau_limit(run_meta: Optional[Dict[str, Any]]) -> Optional[float]:
@@ -92,6 +128,7 @@ def run(
     samples = []
     abort_info = None
 
     # tau_limit for saturation proxy metrics
     tau_limit = _extract_tau_limit(run_meta)
 
@@ -103,6 +140,7 @@ def run(
     try:
         raw_obs0 = sim.get_observations()
         obs0 = adapt_obs(raw_obs0)
@@ -112,14 +150,27 @@ def run(
         joint_commands0 = controller.step(obs0)
         joint_commands0 = normalize_joint_commands(joint_commands0)
         sim.apply_hybrid_command(joint_commands0)
         # Record that we applied initial command
-        samples.append({"t": float(obs0.t), "status": "INIT_CMD"})
+        roll0, pitch0, yaw0 = _quat_to_rpy(getattr(obs0, "base_orientation", (0,0,0,1)))
+        s0 = {"t": float(obs0.t), "status": "INIT_CMD"}
+        qe0 = _q_err_rms(obs0, controller)
+        if qe0 is not None:
+            s0["q_err_rms"] = float(qe0)
+        if roll0 is not None:
+            s0["roll"] = float(roll0)
+        if pitch0 is not None:
+            s0["pitch"] = float(pitch0)
+        if hasattr(obs0, "base_position") and obs0.base_position is not None:
+            try:
+                s0["base_z"] = float(obs0.base_position[2])
+            except Exception:
+                pass
+        samples.append(s0)
     except Exception as e:
         # If something goes wrong here, we still continue; main loop will try again.
         samples.append({"t": 0.0, "status": "INIT_CMD_FAIL", "error": str(e)[:120]})
 
@@ -155,6 +206,7 @@ def run(
             obs = adapt_obs(raw_obs)
             joint_commands = controller.step(obs)
             joint_commands = normalize_joint_commands(joint_commands)
 
             # Saturation proxy stats (near-limit fraction)
             tau_clip_frac, tau_clip_max_ratio = _compute_tau_clip_stats(
                 joint_commands, tau_limit=tau_limit, eps_ratio=0.98
             )
 
             sim.apply_hybrid_command(joint_commands)
 
             # record sample (minimal)
-            s = {"t": float(obs.t), "status": "RUN"}
+            s = {"t": float(obs.t), "status": "RUN"}
+            qe = _q_err_rms(obs, controller)
+            if qe is not None:
+                s["q_err_rms"] = float(qe)
+            r, p, _ = _quat_to_rpy(getattr(obs, "base_orientation", (0,0,0,1)))
+            if r is not None:
+                s["roll"] = float(r)
+            if p is not None:
+                s["pitch"] = float(p)
+            if hasattr(obs, "base_position") and obs.base_position is not None:
+                try:
+                    s["base_z"] = float(obs.base_position[2])
+                except Exception:
+                    pass
             if tau_clip_frac is not None:
                 s["tau_clip_frac"] = float(tau_clip_frac)
             if tau_clip_max_ratio is not None:
                 s["tau_clip_max_ratio"] = float(tau_clip_max_ratio)
             samples.append(s)
```

---

# 2) `src/ext_metrics.py`（q_err/roll/pitch/base_z を集計）

```diff
--- a/src/ext_metrics.py
+++ b/src/ext_metrics.py
@@ -1,12 +1,13 @@
 from __future__ import annotations
 
 from typing import Any, Dict, List, Optional
 import math
 
 
 def compute_metrics(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
@@ -14,6 +15,53 @@
     metrics: Dict[str, Any] = {}
 
+    # --- Tracking / posture metrics (optional) ---
+    qerrs = []
+    rolls = []
+    pitches = []
+    base_zs = []
+    for s in samples or []:
+        if not isinstance(s, dict):
+            continue
+        if "q_err_rms" in s:
+            try:
+                qerrs.append(float(s["q_err_rms"]))
+            except Exception:
+                pass
+        if "roll" in s:
+            try:
+                rolls.append(float(s["roll"]))
+            except Exception:
+                pass
+        if "pitch" in s:
+            try:
+                pitches.append(float(s["pitch"]))
+            except Exception:
+                pass
+        if "base_z" in s:
+            try:
+                base_zs.append(float(s["base_z"]))
+            except Exception:
+                pass
+
+    if qerrs:
+        metrics["q_err_rms_mean"] = float(sum(qerrs) / len(qerrs))
+        metrics["q_err_rms_min"] = float(min(qerrs))
+        metrics["q_err_rms_max"] = float(max(qerrs))
+    if rolls:
+        metrics["roll_abs_max"] = float(max(abs(x) for x in rolls))
+    if pitches:
+        metrics["pitch_abs_max"] = float(max(abs(x) for x in pitches))
+    if base_zs:
+        metrics["base_z_min_sampled"] = float(min(base_zs))
+
     # --- Saturation proxy metrics (optional; present only if ext_runner logs them) ---
     clip_fracs = []
     clip_ratios = []
     for s in samples or []:
```

---

# 3) `scripts/compare_runs.py`（qerr/roll/pitch を表示＋ sort-by qerr を追加）

```diff
--- a/scripts/compare_runs.py
+++ b/scripts/compare_runs.py
@@ -18,6 +18,10 @@
 class RunSummary:
@@
     tau_clip_frac_mean: Optional[float]
     tau_clip_frac_max: Optional[float]
+    q_err_rms_mean: Optional[float]
+    q_err_rms_min: Optional[float]
+    roll_abs_max: Optional[float]
+    pitch_abs_max: Optional[float]
     survival_time: Optional[float]
@@
 def _summarize_run(path: str) -> RunSummary:
@@
     clip_mean = _safe_get(metrics, ["tau_clip_frac_mean"])
     clip_max = _safe_get(metrics, ["tau_clip_frac_max"])
+    qerr_mean = _safe_get(metrics, ["q_err_rms_mean"])
+    qerr_min = _safe_get(metrics, ["q_err_rms_min"])
+    roll_abs_max = _safe_get(metrics, ["roll_abs_max"])
+    pitch_abs_max = _safe_get(metrics, ["pitch_abs_max"])
@@
     try:
         clip_max = float(clip_max) if clip_max is not None else None
     except Exception:
         clip_max = None
+    try:
+        qerr_mean = float(qerr_mean) if qerr_mean is not None else None
+    except Exception:
+        qerr_mean = None
+    try:
+        qerr_min = float(qerr_min) if qerr_min is not None else None
+    except Exception:
+        qerr_min = None
+    try:
+        roll_abs_max = float(roll_abs_max) if roll_abs_max is not None else None
+    except Exception:
+        roll_abs_max = None
+    try:
+        pitch_abs_max = float(pitch_abs_max) if pitch_abs_max is not None else None
+    except Exception:
+        pitch_abs_max = None
 
     return RunSummary(
@@
         tau_clip_frac_mean=clip_mean,
         tau_clip_frac_max=clip_max,
+        q_err_rms_mean=qerr_mean,
+        q_err_rms_min=qerr_min,
+        roll_abs_max=roll_abs_max,
+        pitch_abs_max=pitch_abs_max,
         survival_time=float(st) if st is not None else None,
@@
 def _print_table(runs: List[RunSummary], limit: int) -> None:
     headers = [
@@
         "clip_mean",
         "clip_max",
+        "qerr_mean",
+        "qerr_min",
+        "roll_max",
+        "pitch_max",
         "survival_s",
@@
     rows = []
@@
             _fmt(r.tau_clip_frac_mean, 3),
             _fmt(r.tau_clip_frac_max, 3),
+            _fmt(r.q_err_rms_mean, 3),
+            _fmt(r.q_err_rms_min, 3),
+            _fmt(r.roll_abs_max, 3),
+            _fmt(r.pitch_abs_max, 3),
             _fmt(r.survival_time, 3),
@@
 def main() -> int:
@@
     ap.add_argument(
         "--sort-by",
         default="mtime",
-        choices=["mtime", "survival", "score", "tilt", "energy", "clip"],
+        choices=["mtime", "survival", "score", "tilt", "energy", "clip", "qerr"],
         help="Sort criterion (default: newest first)",
     )
@@
     elif args.sort_by == "clip":
@@
         )
+    elif args.sort_by == "qerr":
+        # Lower tracking error is better; tie-break by score then newest
+        runs.sort(
+            key=lambda r: (
+                r.q_err_rms_mean is None, (r.q_err_rms_mean or 1e9),
+                r.q_err_rms_min is None, (r.q_err_rms_min or 1e9),
+                r.score is None, -(r.score or 0.0),
+                -r.mtime,
+            )
+        )
```

---

## これを入れたら最初にやるコマンド（あなたの position-only 切り分けそのまま）

（目的：**q_err_rms が下がっているかを見る**）

```bash
cd /workspace/hunter

python3 scripts/sweep_tuning.py \
  --mode grid --grid-sample spread --trials 8 --repeats 1 --no-gui \
  --control-dt 0.01 --settle 300 \
  --warmup 999 --blend 0.0 --grav 0 \
  --kp 20 --kd 4.5 --kd-blend 3.0 --tau 150 \
  --pos-kp 60,120,240,480 --pos-kd 1.0,2.0

# 追従できている順に見る（qerr小さい順）
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by qerr --limit 15
```

---

## ここで分岐（重要）

* **qerr_mean が大きいまま／min も下がらない**
  → 「position制御が効いてない」側。次は

  1. **q_ref を crouch（膝曲げ・腰下げ）に変更**
  2. **BASE_HEIGHT を少し下げる**（腰高で初期から倒れてる可能性）
     をやるのが最短です（パッチ出します）。
* **qerr が下がるのに tilt で落ちる**
  → 「目標姿勢が不安定 or 接触で滑る」側。次は crouch 化が本命。

この `q_err_rms` パッチを当てた状態での `compare_runs --sort-by qerr` の上位1〜2本（qerr/roll/pitch/survival）が分かれば、次のパッチ（crouch姿勢 or BASE_HEIGHT調整）を“外さず”出せます。

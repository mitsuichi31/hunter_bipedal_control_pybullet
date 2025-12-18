```diff
--- a/src/ext_runner.py
+++ b/src/ext_runner.py
@@ -1,8 +1,9 @@
 from ext_obs_adapter import adapt_obs
-from ext_safety import should_abort
+from ext_safety import should_abort
 from ext_normalize import normalize_joint_commands
 from ext_metrics import compute_metrics
 
 import json
 import os
 import time as _time
+from typing import Optional, Dict, Any
 
 
 def _ensure_dir(path: str) -> None:
     os.makedirs(path, exist_ok=True)
@@ -28,7 +29,7 @@
 def _tau_from_normalized_cmds(norm_cmds):
     """
     Extract tau dict when commands are torque/hybrid.
     For position commands, returns {}.
     """
@@ -46,6 +47,7 @@
 def run(
     sim,
     controller,
     *,
     seconds: float,
     control_dt: float,
     settle_steps: int = 0,
     log_dir: str = "runs",
     run_name: str = "standing_pd_ext",
+    safety_cfg: Optional[Dict[str, Any]] = None,
 ):
     raw = sim.get_observations()
     obs = adapt_obs(raw)
     if hasattr(controller, "reset"):
         controller.reset(obs)
@@ -67,6 +69,9 @@
     # Optional settling period after reset (lets contacts stabilize)
     for _ in range(max(0, int(settle_steps))):
         sim.step()
         steps += 1
 
     abort_info = None
+    # Safety thresholds are passed through to should_abort(obs, **kwargs)
+    safety_cfg = safety_cfg or {}
 
     while True:
         raw = sim.get_observations()
         obs = adapt_obs(raw)
 
         if obs.t >= end_t:
             break
 
-        abort, reason = should_abort(obs)
+        abort, reason = should_abort(obs, **safety_cfg)
         if abort:
             abort_info = {"t": float(obs.t), "reason": reason}
             break
```

```diff
--- a/src/main_simulation.py
+++ b/src/main_simulation.py
@@ -1,6 +1,7 @@
 import argparse
 import os
+import sys
 import time
 import numpy as np
 
@@ -210,6 +211,7 @@
 def run_standing_pd_ext(duration: float = 10.0, use_gui: bool = True):
@@
     from ext_standing_ref import standing_q_ref
     from ext_pd_posture_torque import PDPostureTorque, TorquePD
     from ext_runner import run
     import yaml
 
@@
     runner_cfg = (cfg.get("runner") or {})
     ctrl_cfg = (cfg.get("controller") or {})
+    safety_cfg = (cfg.get("safety") or {})
 
     # Controller gains from YAML (fallback to defaults)
     gains = TorquePD(
         kp=float(ctrl_cfg.get("kp", 40.0)),
         kd=float(ctrl_cfg.get("kd", 1.5)),
         tau_limit=float(ctrl_cfg.get("tau_limit", 60.0)),
     )
     controller = PDPostureTorque(standing_q_ref(), gains=gains)
 
     seconds = float(runner_cfg.get("seconds", duration))
     control_dt = float(runner_cfg.get("control_dt", 0.01))
     settle_steps = int(runner_cfg.get("settle_steps", 0))
     log_dir = str(runner_cfg.get("log_dir", "runs"))
     run_name = str(runner_cfg.get("run_name", "standing_pd_ext"))
 
     result = run(
         sim,
         controller,
         seconds=seconds,
         control_dt=control_dt,
         settle_steps=settle_steps,
         log_dir=log_dir,
         run_name=run_name,
+        safety_cfg=safety_cfg,
     )
     print(result)
```

```diff
--- /dev/null
+++ b/scripts/run_and_compare.py
@@ -0,0 +1,116 @@
+#!/usr/bin/env python3
+from __future__ import annotations
+
+import argparse
+import os
+import subprocess
+import sys
+
+
+def _run(cmd: list[str], cwd: str | None = None) -> int:
+    p = subprocess.run(cmd, cwd=cwd)
+    return p.returncode
+
+
+def main() -> int:
+    ap = argparse.ArgumentParser(
+        description="Run standing-pd-ext once, then compare recent logs."
+    )
+    ap.add_argument("--duration", type=float, default=10.0, help="Simulation duration seconds")
+    ap.add_argument("--gui", action="store_true", help="Enable GUI (default: off)")
+    ap.add_argument("--log-dir", default="runs", help="Directory containing run logs")
+    ap.add_argument("--prefix", default="standing_pd_ext", help="Log filename prefix")
+    ap.add_argument("--limit", type=int, default=10, help="How many runs to show")
+    ap.add_argument(
+        "--sort-by",
+        default="survival",
+        choices=["mtime", "survival", "tilt", "energy"],
+        help="How to rank runs in compare table",
+    )
+    args = ap.parse_args()
+
+    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
+    src_dir = os.path.join(repo_root, "src")
+
+    # 1) Run one experiment
+    run_cmd = [
+        sys.executable,
+        "main_simulation.py",
+        "--mode",
+        "standing-pd-ext",
+        "--duration",
+        str(args.duration),
+    ]
+    if not args.gui:
+        run_cmd.append("--no-gui")
+
+    print("\n== Running experiment ==")
+    rc = _run(run_cmd, cwd=src_dir)
+    if rc != 0:
+        print(f"Experiment failed with exit code {rc}")
+        return rc
+
+    # 2) Compare logs
+    print("\n== Comparing logs ==")
+    compare_cmd = [
+        sys.executable,
+        os.path.join("scripts", "compare_runs.py"),
+        "--log-dir",
+        args.log_dir,
+        "--prefix",
+        args.prefix,
+        "--limit",
+        str(args.limit),
+        "--sort-by",
+        args.sort_by,
+    ]
+    rc2 = _run(compare_cmd, cwd=repo_root)
+    return rc2
+
+
+if __name__ == "__main__":
+    raise SystemExit(main())
```

使い方（Dockerコンテナ内）:

```bash
cd /workspace/hunter
python3 scripts/run_and_compare.py --duration 10 --limit 10 --sort-by survival
```

GUIありなら:

```bash
cd /workspace/hunter
QT_X11_NO_MITSHM=1 PYBULLET_USE_OPENGL2=1 \
python3 scripts/run_and_compare.py --duration 10 --gui
```

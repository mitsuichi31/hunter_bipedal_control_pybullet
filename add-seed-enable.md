```diff
--- a/config/agent_tuning.yaml
+++ b/config/agent_tuning.yaml
@@ -1,10 +1,13 @@
 # Agent-tunable parameters for standing-pd-ext
 
 runner:
   seconds: 10.0
   # Controller update period. 0.01 = 100 Hz, 0.001 = 1 kHz
   control_dt: 0.01
+  # Random seed for reproducibility (used by standing-pd-ext)
+  seed: 0
   # Number of physics ticks to wait after reset before starting control
   settle_steps: 300
   log_dir: "runs"
   run_name: "standing_pd_ext"
```

```diff
--- a/src/main_simulation.py
+++ b/src/main_simulation.py
@@ -1,6 +1,8 @@
 import argparse
 import os
 import sys
+import random
 import time
 import numpy as np
 
@@ -210,6 +212,7 @@
 def run_standing_pd_ext(duration: float = 10.0, use_gui: bool = True):
@@
     from ext_standing_ref import standing_q_ref
     from ext_pd_posture_torque import PDPostureTorque, TorquePD
     from ext_runner import run
     import yaml
 
@@
     runner_cfg = (cfg.get("runner") or {})
     ctrl_cfg = (cfg.get("controller") or {})
     safety_cfg = (cfg.get("safety") or {})
 
+    # Seed (reproducibility)
+    seed = int(runner_cfg.get("seed", 0) or 0)
+    random.seed(seed)
+    np.random.seed(seed)
+
     q_ref = standing_q_ref()
 
     # Controller selection
     ctrl_type = str(ctrl_cfg.get("type", "torque_pd"))
@@ -270,6 +279,7 @@
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
         safety_cfg=safety_cfg,
+        run_meta={"seed": seed, "runner": runner_cfg, "controller": ctrl_cfg, "safety": safety_cfg},
     )
     print(result)
```

```diff
--- a/src/ext_runner.py
+++ b/src/ext_runner.py
@@ -1,9 +1,10 @@
 from ext_obs_adapter import adapt_obs
 from ext_safety import should_abort
 from ext_normalize import normalize_joint_commands
 from ext_metrics import compute_metrics
 
 import json
 import os
 import time as _time
-from typing import Optional, Dict, Any
+from typing import Optional, Dict, Any
 
 
 def _ensure_dir(path: str) -> None:
     os.makedirs(path, exist_ok=True)
@@ -54,6 +55,7 @@
 def run(
     sim,
     controller,
     *,
     seconds: float,
     control_dt: float,
     settle_steps: int = 0,
     log_dir: str = "runs",
     run_name: str = "standing_pd_ext",
     safety_cfg: Optional[Dict[str, Any]] = None,
+    run_meta: Optional[Dict[str, Any]] = None,
 ):
@@ -120,6 +122,7 @@
     metrics = compute_metrics(samples)
 
     result = {
         "status": "ABORT" if abort_info else "DONE",
         "t": float(obs.t),
         "steps": steps,
         "updates": updates,
         "abort": abort_info,
         "metrics": metrics,
     }
 
     # Save run JSON (agent reads these)
     _ensure_dir(log_dir)
     path = os.path.join(log_dir, f"{run_name}_{_now_tag()}.json")
     payload = {
+        "meta": run_meta or {},
         "result": result,
         "samples": samples,  # can be large; keep for now for debugging/agent learning
     }
     with open(path, "w", encoding="utf-8") as f:
         json.dump(payload, f, ensure_ascii=False, indent=2)
 
     result["log_path"] = path
     return result
```

### 使い方（Docker内）

```bash
cd /workspace/hunter

# seed=0 のまま 3回回してログの meta.seed を確認
python3 scripts/run_and_compare.py --duration 3 --sort-by score

# repeats sweep では seed が base_seed + r で変わる（sweep_tuning.py が既にそう書いている）
python3 scripts/sweep_tuning.py --mode grid --trials 3 --warmup 0.5,1.0 --repeats 2 --no-gui
```

ログ（runs/*.json）には `meta.seed` と `meta.runner/meta.controller/meta.safety` が入るようになります。

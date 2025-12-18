```diff
--- /dev/null
+++ b/src/ext_controller_two_stage.py
@@ -0,0 +1,176 @@
+from __future__ import annotations
+
+from dataclasses import dataclass
+from typing import Dict, Any, Optional
+import numpy as np
+
+from ext_joints import LEG_JOINTS
+
+
+@dataclass
+class PositionStageGains:
+    kp: float = 0.3
+    kd: float = 0.1
+
+
+@dataclass
+class TorqueStageGains:
+    kp: float = 40.0
+    kd: float = 1.5
+    tau_limit: float = 60.0
+
+
+class TwoStagePostureController:
+    """
+    Stage 1 (warmup): position control to quickly establish posture
+    Stage 2: torque PD to maintain posture
+
+    Output format matches HunterSimulation.apply_hybrid_command():
+      - position: {"mode":"position","value":..., "kp":..., "kd":...}
+      - torque:   {"mode":"torque","value":...}
+    """
+
+    def __init__(
+        self,
+        q_ref: np.ndarray,
+        *,
+        warmup_seconds: float = 1.0,
+        position_gains: PositionStageGains = PositionStageGains(),
+        torque_gains: TorqueStageGains = TorqueStageGains(),
+    ):
+        assert q_ref.shape == (10,)
+        self.q_ref = q_ref.astype(float)
+        self.warmup_seconds = float(warmup_seconds)
+        self.pos_g = position_gains
+        self.tau_g = torque_gains
+        self._t0: Optional[float] = None
+
+    def reset(self, obs) -> None:
+        self._t0 = float(obs.t)
+
+    def _in_warmup(self, t: float) -> bool:
+        if self._t0 is None:
+            self._t0 = t
+        return (t - self._t0) < self.warmup_seconds
+
+    def step(self, obs) -> Dict[str, Any]:
+        t = float(obs.t)
+
+        if self._in_warmup(t):
+            # Strong posture acquisition using POSITION_CONTROL
+            cmds: Dict[str, Any] = {}
+            for j, qd in zip(LEG_JOINTS, self.q_ref):
+                cmds[j] = {
+                    "mode": "position",
+                    "value": float(qd),
+                    "kp": float(self.pos_g.kp),
+                    "kd": float(self.pos_g.kd),
+                }
+            return cmds
+
+        # Maintain using torque PD (no gravity comp here; add later if needed)
+        tau = self.tau_g.kp * (self.q_ref - obs.q) - self.tau_g.kd * obs.dq
+        tau = np.clip(tau, -self.tau_g.tau_limit, self.tau_g.tau_limit)
+        return {j: {"mode": "torque", "value": float(t)} for j, t in zip(LEG_JOINTS, tau)}
```

```diff
--- a/config/agent_tuning.yaml
+++ b/config/agent_tuning.yaml
@@ -1,28 +1,44 @@
 # Agent-tunable parameters for standing-pd-ext
 
 runner:
   seconds: 10.0
   # Controller update period. 0.01 = 100 Hz, 0.001 = 1 kHz
   control_dt: 0.01
   # Number of physics ticks to wait after reset before starting control
   settle_steps: 300
   log_dir: "runs"
   run_name: "standing_pd_ext"
 
 safety:
   max_roll: 0.7
   max_pitch: 0.7
   min_base_z: 0.12
   max_omega: 20.0
 
 controller:
-  # For PDPostureTorque
-  kp: 40.0
-  kd: 1.5
-  tau_limit: 60.0
+  # Select controller implementation:
+  # - "torque_pd": torque-only PD posture hold (harder to stand up)
+  # - "two_stage": position warmup then torque PD hold (recommended)
+  type: "two_stage"
+
+  # Only used when type == "two_stage"
+  warmup_seconds: 1.0
+
+  # Warmup stage (POSITION_CONTROL)
+  position:
+    kp: 0.3
+    kd: 0.1
+
+  # Hold stage (TORQUE_CONTROL PD)
+  torque:
+    kp: 40.0
+    kd: 1.5
+    tau_limit: 60.0
```

```diff
--- a/src/main_simulation.py
+++ b/src/main_simulation.py
@@ -210,7 +210,7 @@
 def run_standing_pd_ext(duration: float = 10.0, use_gui: bool = True):
@@
-    from ext_standing_ref import standing_q_ref
-    from ext_pd_posture_torque import PDPostureTorque, TorquePD
+    from ext_standing_ref import standing_q_ref
+    from ext_pd_posture_torque import PDPostureTorque, TorquePD
     from ext_runner import run
     import yaml
 
@@
-    runner_cfg = (cfg.get("runner") or {})
-    ctrl_cfg = (cfg.get("controller") or {})
+    runner_cfg = (cfg.get("runner") or {})
+    ctrl_cfg = (cfg.get("controller") or {})
     safety_cfg = (cfg.get("safety") or {})
 
-    # Controller gains from YAML (fallback to defaults)
-    gains = TorquePD(
-        kp=float(ctrl_cfg.get("kp", 40.0)),
-        kd=float(ctrl_cfg.get("kd", 1.5)),
-        tau_limit=float(ctrl_cfg.get("tau_limit", 60.0)),
-    )
-    controller = PDPostureTorque(standing_q_ref(), gains=gains)
+    q_ref = standing_q_ref()
+
+    # Controller selection
+    ctrl_type = str(ctrl_cfg.get("type", "torque_pd"))
+    if ctrl_type == "two_stage":
+        from ext_controller_two_stage import (
+            TwoStagePostureController,
+            PositionStageGains,
+            TorqueStageGains,
+        )
+        warmup_seconds = float(ctrl_cfg.get("warmup_seconds", 1.0))
+        pos_cfg = (ctrl_cfg.get("position") or {})
+        tau_cfg = (ctrl_cfg.get("torque") or {})
+        controller = TwoStagePostureController(
+            q_ref,
+            warmup_seconds=warmup_seconds,
+            position_gains=PositionStageGains(
+                kp=float(pos_cfg.get("kp", 0.3)),
+                kd=float(pos_cfg.get("kd", 0.1)),
+            ),
+            torque_gains=TorqueStageGains(
+                kp=float(tau_cfg.get("kp", 40.0)),
+                kd=float(tau_cfg.get("kd", 1.5)),
+                tau_limit=float(tau_cfg.get("tau_limit", 60.0)),
+            ),
+        )
+    else:
+        # torque_pd (backward compatible: allow flat kp/kd/tau_limit in YAML)
+        gains = TorquePD(
+            kp=float(ctrl_cfg.get("kp", (ctrl_cfg.get("torque") or {}).get("kp", 40.0))),
+            kd=float(ctrl_cfg.get("kd", (ctrl_cfg.get("torque") or {}).get("kd", 1.5))),
+            tau_limit=float(ctrl_cfg.get("tau_limit", (ctrl_cfg.get("torque") or {}).get("tau_limit", 60.0))),
+        )
+        controller = PDPostureTorque(q_ref, gains=gains)
 
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
     )
     print(result)
```

```diff
--- a/scripts/sweep_tuning.py
+++ b/scripts/sweep_tuning.py
@@ -1,6 +1,7 @@
 #!/usr/bin/env python3
 from __future__ import annotations
 
 import argparse
 import itertools
 import json
@@ -116,6 +117,7 @@
     ap.add_argument("--seed", type=int, default=0, help="Random seed (random mode)")
     ap.add_argument("--duration", type=float, default=None, help="Override runner.seconds (optional)")
     ap.add_argument("--no-gui", action="store_true", help="Force --no-gui for main_simulation.py")
     ap.add_argument("--dry-run", action="store_true", help="Print planned trials but do not run")
+    ap.add_argument("--warmup", default="1.0", help="Comma-separated warmup_seconds candidates (two_stage)")
 
     # Coarse, safe-ish defaults (good enough to start agent loop)
     ap.add_argument("--kp", default="20,40,60,80,100,120", help="Comma-separated kp candidates (grid)")
     ap.add_argument("--kd", default="0.5,1.0,1.5,2.0,3.0,4.0", help="Comma-separated kd candidates (grid)")
     ap.add_argument("--tau", default="30,60,90,120", help="Comma-separated tau_limit candidates (grid)")
     ap.add_argument("--control-dt", default="0.01,0.001", help="Comma-separated control_dt candidates (grid)")
     ap.add_argument("--settle", default="0,300,800", help="Comma-separated settle_steps candidates (grid)")
     args = ap.parse_args()
@@ -133,6 +135,7 @@
     def parse_floats(s: str) -> List[float]:
         return [float(x.strip()) for x in s.split(",") if x.strip()]
 
     kps = _mk_grid(parse_floats(args.kp))
     kds = _mk_grid(parse_floats(args.kd))
     taus = _mk_grid(parse_floats(args.tau))
     dts = _mk_grid(parse_floats(args.control_dt))
     settles = [int(float(x)) for x in args.settle.split(",") if x.strip()]
+    warmups = _mk_grid(parse_floats(args.warmup))
 
     planned_params: List[Dict[str, Any]] = []
 
     if args.mode == "grid":
         # Keep grid manageable by subsampling if too large
-        all_params = list(itertools.product(kps, kds, taus, dts, settles))
+        all_params = list(itertools.product(kps, kds, taus, dts, settles, warmups))
         # If too large, take a deterministic subset spread across the space
         max_grid = max(1, args.trials)
         if len(all_params) > max_grid:
             step = max(1, len(all_params) // max_grid)
             all_params = all_params[::step][:max_grid]
-        for kp, kd, tau, dt, settle in all_params:
+        for kp, kd, tau, dt, settle, warmup in all_params:
             planned_params.append(
-                {"kp": kp, "kd": kd, "tau_limit": tau, "control_dt": dt, "settle_steps": settle}
+                {
+                    "kp": kp,
+                    "kd": kd,
+                    "tau_limit": tau,
+                    "control_dt": dt,
+                    "settle_steps": settle,
+                    "warmup_seconds": warmup,
+                }
             )
     else:
         random.seed(args.seed)
         for _ in range(args.trials):
             planned_params.append(
                 {
                     "kp": random.choice(kps),
                     "kd": random.choice(kds),
                     "tau_limit": random.choice(taus),
                     "control_dt": random.choice(dts),
                     "settle_steps": random.choice(settles),
+                    "warmup_seconds": random.choice(warmups),
                 }
             )
@@ -186,6 +189,7 @@
         cfg = dict(base_cfg)
         cfg_runner = dict(runner_cfg)
         cfg_ctrl = dict(ctrl_cfg)
 
         if args.duration is not None:
             cfg_runner["seconds"] = float(args.duration)
         cfg_runner["control_dt"] = float(p["control_dt"])
         cfg_runner["settle_steps"] = int(p["settle_steps"])
 
-        cfg_ctrl["kp"] = float(p["kp"])
-        cfg_ctrl["kd"] = float(p["kd"])
-        cfg_ctrl["tau_limit"] = float(p["tau_limit"])
+        # Support both controller layouts:
+        #  - legacy: controller.{kp,kd,tau_limit}
+        #  - two_stage: controller.type == "two_stage" and controller.torque.{kp,kd,tau_limit}
+        ctrl_type = str(cfg_ctrl.get("type", "torque_pd"))
+        if ctrl_type == "two_stage":
+            cfg_ctrl["warmup_seconds"] = float(p.get("warmup_seconds", cfg_ctrl.get("warmup_seconds", 1.0)))
+            torque_block = dict(cfg_ctrl.get("torque") or {})
+            torque_block["kp"] = float(p["kp"])
+            torque_block["kd"] = float(p["kd"])
+            torque_block["tau_limit"] = float(p["tau_limit"])
+            cfg_ctrl["torque"] = torque_block
+        else:
+            cfg_ctrl["kp"] = float(p["kp"])
+            cfg_ctrl["kd"] = float(p["kd"])
+            cfg_ctrl["tau_limit"] = float(p["tau_limit"])
 
         cfg["runner"] = cfg_runner
         cfg["controller"] = cfg_ctrl
@@ -279,6 +289,14 @@
     best_cfg = _load_yaml(os.path.join(repo_root, args.config))
     best_cfg_runner = best_cfg.get("runner") or {}
     best_cfg_ctrl = best_cfg.get("controller") or {}
     best_cfg_runner["control_dt"] = float(best.params["control_dt"])
     best_cfg_runner["settle_steps"] = int(best.params["settle_steps"])
-    best_cfg_ctrl["kp"] = float(best.params["kp"])
-    best_cfg_ctrl["kd"] = float(best.params["kd"])
-    best_cfg_ctrl["tau_limit"] = float(best.params["tau_limit"])
+    if str(best_cfg_ctrl.get("type", "torque_pd")) == "two_stage":
+        best_cfg_ctrl["warmup_seconds"] = float(best.params.get("warmup_seconds", best_cfg_ctrl.get("warmup_seconds", 1.0)))
+        torque_block = dict(best_cfg_ctrl.get("torque") or {})
+        torque_block["kp"] = float(best.params["kp"])
+        torque_block["kd"] = float(best.params["kd"])
+        torque_block["tau_limit"] = float(best.params["tau_limit"])
+        best_cfg_ctrl["torque"] = torque_block
+    else:
+        best_cfg_ctrl["kp"] = float(best.params["kp"])
+        best_cfg_ctrl["kd"] = float(best.params["kd"])
+        best_cfg_ctrl["tau_limit"] = float(best.params["tau_limit"])
     best_cfg["runner"] = best_cfg_runner
     best_cfg["controller"] = best_cfg_ctrl
```

### 実行例（Dockerコンテナ内）

```bash
cd /workspace/hunter

# まずは two_stage のまま（agent_tuning.yaml の controller.type が two_stage）
python3 scripts/run_and_compare.py --duration 10 --sort-by survival

# 探索：維持（torque段）のゲインを中心に sweep（最大24試行に間引き）
python3 scripts/sweep_tuning.py --mode grid --trials 24 --no-gui

# warmup_seconds も探索したい場合（例: 0.5,1.0,1.5）
python3 scripts/sweep_tuning.py --mode grid --trials 30 --warmup 0.5,1.0,1.5 --no-gui

# 結果を見る
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by survival --limit 10
```

この状態になると、エージェントは主に

* `config/agent_tuning.yaml`（探索）
* `runs/*.json`（観測）
  だけで改善ループを回せます。

以下が「**two_stage の warmup→hold 切替をブレンディング（段階的移行）**」するための **次ステップ・パッチ一式（unified diff）**です。
（前提：あなたの現状は *two_stage + gravity_comp* が入っていて、`main_simulation.py` の standing-pd-ext で YAML の controller を読んで TwoStage を生成しています。）

---

## 1) `src/ext_controller_two_stage.py`（ブレンディング実装）

* `blend_seconds` を追加
* warmup 後に `blend_seconds` だけ **mode="hybrid"** を使って滑らかに移行

  * position目標は維持しつつ、`tau_ff = alpha * (tau_pd + gravity_ff)` を足していく
* hold は従来どおり torque PD（+gravity_ff）

```diff
--- a/src/ext_controller_two_stage.py
+++ b/src/ext_controller_two_stage.py
@@ -1,14 +1,15 @@
 from __future__ import annotations
 
 from dataclasses import dataclass
 from typing import Dict, Any, Optional
 import numpy as np
 
 from ext_joints import LEG_JOINTS
 from gravity_compensation import GravityCompensation
 
 
 @dataclass
 class PositionStageGains:
     kp: float = 0.3
     kd: float = 0.1
 
 
 @dataclass
 class TorqueStageGains:
     kp: float = 40.0
     kd: float = 1.5
     tau_limit: float = 60.0
     use_gravity_comp: bool = False
     gravity_scale: float = 1.0
 
 
 class TwoStagePostureController:
     """
     Warmup: position control to reach q_ref (wide stable stance).
+    Blend:  smooth transition using hybrid command (position PD + ramped torque FF).
     Hold:   torque PD (+ optional gravity feedforward).
     """
 
     def __init__(
         self,
         q_ref: np.ndarray,
         *,
         robot_id: int,
         warmup_seconds: float = 1.0,
+        blend_seconds: float = 0.0,
         position_gains: PositionStageGains = PositionStageGains(),
         torque_gains: TorqueStageGains = TorqueStageGains(),
     ):
         assert q_ref.shape == (10,)
         self.q_ref = q_ref.astype(float)
         self.warmup_seconds = float(warmup_seconds)
+        self.blend_seconds = max(0.0, float(blend_seconds))
         self.pos_g = position_gains
         self.tau_g = torque_gains
         self._t0: Optional[float] = None
         self.robot_id = int(robot_id)
 
         # Create once; GravityCompensation caches joint info internally.
         self._gc = GravityCompensation(self.robot_id)
 
     def reset(self, obs) -> None:
         self._t0 = float(obs.t)
 
     def _in_warmup(self, t: float) -> bool:
         if self._t0 is None:
             return True
         return (t - self._t0) < self.warmup_seconds
 
+    def _in_blend(self, t: float) -> bool:
+        if self._t0 is None:
+            return False
+        if self.blend_seconds <= 0.0:
+            return False
+        dt = (t - self._t0)
+        return (dt >= self.warmup_seconds) and (dt < (self.warmup_seconds + self.blend_seconds))
+
+    def _blend_alpha(self, t: float) -> float:
+        """0 -> 1 across the blend window."""
+        if not self._in_blend(t):
+            return 1.0
+        dt = (t - self._t0) - self.warmup_seconds
+        return float(np.clip(dt / max(1e-9, self.blend_seconds), 0.0, 1.0))
+
     def _gravity_ff(self, obs) -> np.ndarray:
         """
         Gravity feedforward torque for LEG_JOINTS in controller order.
         Uses dict interface to avoid any joint ordering mismatch.
         """
         if not self.tau_g.use_gravity_comp:
             return np.zeros(10, dtype=float)
 
         q_dict = {name: float(qi) for name, qi in zip(LEG_JOINTS, obs.q)}
         tau_dict = self._gc.compute_gravity_torques_dict(joint_positions=q_dict)
         tau = np.array([float(tau_dict.get(name, 0.0)) for name in LEG_JOINTS], dtype=float)
         return tau * float(self.tau_g.gravity_scale)
 
     def step(self, obs) -> Dict[str, Any]:
         t = float(obs.t)
         if self._in_warmup(t):
             # Warmup: position control toward q_ref
             cmds = {}
             for i, j in enumerate(LEG_JOINTS):
                 cmds[j] = {
                     "mode": "position",
                     "value": float(self.q_ref[i]),
                     "kp": float(self.pos_g.kp),
                     "kd": float(self.pos_g.kd),
                 }
             return cmds
 
+        # Blend: position PD + ramped torque feedforward (torque PD + gravity)
+        if self._in_blend(t):
+            a = self._blend_alpha(t)
+            # Torque PD (the same as hold), but ramped in gradually
+            tau_pd = self.tau_g.kp * (self.q_ref - obs.q) - self.tau_g.kd * obs.dq
+            tau_ff = (tau_pd + self._gravity_ff(obs)) * a
+            tau_ff = np.clip(tau_ff, -self.tau_g.tau_limit, self.tau_g.tau_limit)
+
+            cmds = {}
+            for i, j in enumerate(LEG_JOINTS):
+                cmds[j] = {
+                    "mode": "hybrid",
+                    "position": float(self.q_ref[i]),
+                    "velocity": 0.0,
+                    "kp": float(self.pos_g.kp),
+                    "kd": float(self.pos_g.kd),
+                    "torque": float(tau_ff[i]),
+                }
+            return cmds
+
         # Hold: torque PD + optional gravity feedforward
         tau_pd = self.tau_g.kp * (self.q_ref - obs.q) - self.tau_g.kd * obs.dq
         tau = tau_pd + self._gravity_ff(obs)
         tau = np.clip(tau, -self.tau_g.tau_limit, self.tau_g.tau_limit)
         return {j: {"mode": "torque", "value": float(tau_i)} for j, tau_i in zip(LEG_JOINTS, tau)}
```

---

## 2) `src/main_simulation.py`（standing-pd-ext / two_stage に blend_seconds を渡す）

`standing-pd-ext` の two_stage 生成部に `blend_seconds` を追加して渡します。

```diff
--- a/src/main_simulation.py
+++ b/src/main_simulation.py
@@ -1,6 +1,7 @@
         ctrl_type = str(ctrl_cfg.get("type", "torque_pd"))
         if ctrl_type == "two_stage":
             from ext_controller_two_stage import (
                 TwoStagePostureController,
                 PositionStageGains,
                 TorqueStageGains,
             )
 
             warmup_seconds = float(ctrl_cfg.get("warmup_seconds", 1.0))
+            blend_seconds = float(ctrl_cfg.get("blend_seconds", 0.0))
             pos_cfg = (ctrl_cfg.get("position") or {})
             tau_cfg = (ctrl_cfg.get("torque") or {})
             controller = TwoStagePostureController(
                 q_ref,
                 robot_id=sim.robot_id,
                 warmup_seconds=warmup_seconds,
+                blend_seconds=blend_seconds,
                 position_gains=PositionStageGains(
                     kp=float(pos_cfg.get("kp", 0.3)),
                     kd=float(pos_cfg.get("kd", 0.1)),
                 ),
                 torque_gains=TorqueStageGains(
                     kp=float(tau_cfg.get("kp", 40.0)),
                     kd=float(tau_cfg.get("kd", 1.5)),
                     tau_limit=float(tau_cfg.get("tau_limit", 60.0)),
                     use_gravity_comp=bool(tau_cfg.get("use_gravity_comp", False)),
                     gravity_scale=float(tau_cfg.get("gravity_scale", 1.0)),
                 ),
             )
```

---

## 3) `config/agent_tuning.yaml`（blend_seconds を追加）

デフォルトは 0.2 秒くらいを推奨（いったん 0.2 で試す）。
既存に影響を出したくないなら 0.0 にしてもOK（=従来挙動）。

```diff
--- a/config/agent_tuning.yaml
+++ b/config/agent_tuning.yaml
@@ -1,9 +1,13 @@
 controller:
   type: two_stage
 
   warmup_seconds: 1.0
+  # Smooth transition from warmup(position) to hold(torque)
+  # 0.0 means "no blending" (instant switch; old behavior).
+  blend_seconds: 0.2
 
   # Warmup stage (POSITION_CONTROL)
   position:
     kp: 0.3
     kd: 0.1
```

---

## 4) `scripts/sweep_tuning.py`（--blend を追加して探索対象へ）

* `--blend 0,0.1,0.2,0.4` みたいに探索できるようにする
* planned params に `blend_seconds` を入れて YAML に書き込む

```diff
--- a/scripts/sweep_tuning.py
+++ b/scripts/sweep_tuning.py
@@ -1,10 +1,11 @@
     ap.add_argument("--mode", choices=["grid", "random"], default="grid")
     ap.add_argument("--trials", type=int, default=20)
     ap.add_argument("--seed", type=int, default=0)
     ap.add_argument("--no-gui", action="store_true")
     ap.add_argument("--dry-run", action="store_true")
     ap.add_argument("--warmup", default="1.0", help="Comma-separated warmup_seconds candidates (two_stage)")
+    ap.add_argument("--blend", default="0.0", help="Comma-separated blend_seconds candidates (two_stage)")
     ap.add_argument("--grav", default="0,1", help="Comma-separated use_gravity_comp candidates: 0 or 1 (two_stage)")
     ap.add_argument("--grav-scale", default="1.0", help="Comma-separated gravity_scale candidates (two_stage)")
 
@@ -1,6 +1,7 @@
     warmups = _mk_grid(parse_floats(args.warmup))
+    blends = _mk_grid(parse_floats(args.blend))
     grav_flags = [int(float(x)) for x in args.grav.split(",") if x.strip()]
     grav_scales = _mk_grid(parse_floats(args.grav_scale))
 
@@ -1,7 +1,7 @@
     if args.mode == "grid":
-        all_params = list(itertools.product(kps, kds, taus, dts, settles, warmups, grav_flags, grav_scales))
+        all_params = list(itertools.product(kps, kds, taus, dts, settles, warmups, blends, grav_flags, grav_scales))
         max_grid = max(1, args.trials)
         if len(all_params) > max_grid:
             step = max(1, len(all_params) // max_grid)
             all_params = all_params[::step][:max_grid]
-        for kp, kd, tau, dt, settle, warmup, grav, gscale in all_params:
+        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale in all_params:
             planned_params.append(
                 {
                     "kp": kp,
                     "kd": kd,
                     "tau_limit": tau,
                     "control_dt": dt,
                     "settle_steps": settle,
                     "warmup_seconds": warmup,
+                    "blend_seconds": blend,
                     "use_gravity_comp": bool(int(grav)),
                     "gravity_scale": float(gscale),
                 }
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
                     "warmup_seconds": random.choice(warmups),
+                    "blend_seconds": random.choice(blends),
                     "use_gravity_comp": bool(int(random.choice(grav_flags))),
                     "gravity_scale": float(random.choice(grav_scales)),
                 }
             )
 
@@ -1,10 +1,13 @@
         ctrl_type = str(cfg_ctrl.get("type", "torque_pd"))
         if ctrl_type == "two_stage":
             cfg_ctrl["warmup_seconds"] = float(p.get("warmup_seconds", cfg_ctrl.get("warmup_seconds", 1.0)))
+            cfg_ctrl["blend_seconds"] = float(p.get("blend_seconds", cfg_ctrl.get("blend_seconds", 0.0)))
             torque_block = dict(cfg_ctrl.get("torque") or {})
             torque_block["kp"] = float(p["kp"])
             torque_block["kd"] = float(p["kd"])
             torque_block["tau_limit"] = float(p["tau_limit"])
             torque_block["use_gravity_comp"] = bool(p.get("use_gravity_comp", torque_block.get("use_gravity_comp", False)))
             torque_block["gravity_scale"] = float(p.get("gravity_scale", torque_block.get("gravity_scale", 1.0)))
             cfg_ctrl["torque"] = torque_block
 
@@ -1,10 +1,13 @@
     if str(best_cfg_ctrl.get("type", "torque_pd")) == "two_stage":
         best_cfg_ctrl["warmup_seconds"] = float(best.params.get("warmup_seconds", best_cfg_ctrl.get("warmup_seconds", 1.0)))
+        best_cfg_ctrl["blend_seconds"] = float(best.params.get("blend_seconds", best_cfg_ctrl.get("blend_seconds", 0.0)))
         torque_block = dict(best_cfg_ctrl.get("torque") or {})
         torque_block["kp"] = float(best.params["kp"])
         torque_block["kd"] = float(best.params["kd"])
         torque_block["tau_limit"] = float(best.params["tau_limit"])
         torque_block["use_gravity_comp"] = bool(best.params.get("use_gravity_comp", torque_block.get("use_gravity_comp", False)))
         torque_block["gravity_scale"] = float(best.params.get("gravity_scale", torque_block.get("gravity_scale", 1.0)))
         best_cfg_ctrl["torque"] = torque_block
```

---

# 使い方（Docker内）例

### まず「切替が原因で転ぶか」を確認する最小 sweep

```bash
cd /workspace/hunter
python3 scripts/sweep_tuning.py \
  --mode grid --trials 6 \
  --kp 40 --kd 1.5 --tau 60 \
  --control-dt 0.01 --settle 0 \
  --warmup 0.5 \
  --blend 0.0,0.2,0.4 \
  --grav 1 --grav-scale 1.0 \
  --repeats 2 \
  --no-gui
```

### 結果の見方

* `python3 scripts/compare_runs.py --sort-by score --grav on`
* `python3 scripts/analyze_abort_reasons.py --summary runs/sweep_summary.json`

---



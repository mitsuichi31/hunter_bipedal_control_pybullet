以下、「次ステップ（two_stage の hold トルクに重力補償を入れて、YAML / sweep / main_simulation を整合させる）」の **必要パッチ一式（unified diff）**です。`main_simulation.py` の two_stage 生成箇所は、あなたの現状実装（standing-pd-ext）に合わせて最小変更にしています。

---

## 1) `src/ext_controller_two_stage.py`

```diff
--- a/src/ext_controller_two_stage.py
+++ b/src/ext_controller_two_stage.py
@@ -1,22 +1,27 @@
 from __future__ import annotations
 
 from dataclasses import dataclass
-from typing import Dict, Any, Optional
+from typing import Dict, Any, Optional
 import numpy as np
 
 from ext_joints import LEG_JOINTS
+from gravity_compensation import GravityCompensation
 
 
 @dataclass
 class PositionStageGains:
     kp: float = 0.3
     kd: float = 0.1
 
 
 @dataclass
 class TorqueStageGains:
     kp: float = 40.0
     kd: float = 1.5
     tau_limit: float = 60.0
+    # Gravity compensation (optional)
+    use_gravity_comp: bool = False
+    gravity_scale: float = 1.0
 
 
 class TwoStagePostureController:
@@ -25,22 +30,59 @@
     Warmup: position control to reach q_ref (wide stable stance).
     Hold: torque PD to maintain q_ref.
     """
 
     def __init__(
         self,
         q_ref: np.ndarray,
         *,
+        robot_id: int,
         warmup_seconds: float = 1.0,
         position_gains: PositionStageGains = PositionStageGains(),
         torque_gains: TorqueStageGains = TorqueStageGains(),
     ):
         assert q_ref.shape == (10,)
         self.q_ref = q_ref.astype(float)
         self.warmup_seconds = float(warmup_seconds)
         self.pos_g = position_gains
         self.tau_g = torque_gains
         self._t0: Optional[float] = None
+        self.robot_id = int(robot_id)
+
+        # Create once; GravityCompensation caches joint info internally.
+        self._gc = GravityCompensation(self.robot_id)
 
     def reset(self, obs) -> None:
         self._t0 = float(obs.t)
 
     def _in_warmup(self, t: float) -> bool:
         if self._t0 is None:
             return True
         return (t - self._t0) < self.warmup_seconds
 
+    def _gravity_ff(self, obs) -> np.ndarray:
+        """
+        Gravity feedforward torque for LEG_JOINTS in controller order.
+        Uses dict interface to avoid any joint ordering mismatch.
+        """
+        if not self.tau_g.use_gravity_comp:
+            return np.zeros(10, dtype=float)
+
+        q_dict = {name: float(qi) for name, qi in zip(LEG_JOINTS, obs.q)}
+        tau_dict = self._gc.compute_gravity_torques_dict(joint_positions=q_dict)
+        tau = np.array([float(tau_dict.get(name, 0.0)) for name in LEG_JOINTS], dtype=float)
+        return tau * float(self.tau_g.gravity_scale)
+
     def step(self, obs) -> Dict[str, Any]:
         if self._in_warmup(float(obs.t)):
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
 
-        # Hold: torque PD (no gravity comp here; add later if needed)
-        tau = self.tau_g.kp * (self.q_ref - obs.q) - self.tau_g.kd * obs.dq
+        # Hold: torque PD + optional gravity feedforward
+        tau_pd = self.tau_g.kp * (self.q_ref - obs.q) - self.tau_g.kd * obs.dq
+        tau = tau_pd + self._gravity_ff(obs)
         tau = np.clip(tau, -self.tau_g.tau_limit, self.tau_g.tau_limit)
         return {j: {"mode": "torque", "value": float(t)} for j, t in zip(LEG_JOINTS, tau)}
```

---

## 2) `src/main_simulation.py`（standing-pd-ext / two_stage の生成を更新）

```diff
--- a/src/main_simulation.py
+++ b/src/main_simulation.py
@@ -1,70 +1,70 @@
         ctrl_type = str(ctrl_cfg.get("type", "torque_pd"))
         if ctrl_type == "two_stage":
             from ext_controller_two_stage import (
                 TwoStagePostureController,
                 PositionStageGains,
                 TorqueStageGains,
             )
 
             warmup_seconds = float(ctrl_cfg.get("warmup_seconds", 1.0))
             pos_cfg = (ctrl_cfg.get("position") or {})
             tau_cfg = (ctrl_cfg.get("torque") or {})
             controller = TwoStagePostureController(
                 q_ref,
+                robot_id=sim.robot_id,
                 warmup_seconds=warmup_seconds,
                 position_gains=PositionStageGains(
                     kp=float(pos_cfg.get("kp", 0.3)),
                     kd=float(pos_cfg.get("kd", 0.1)),
                 ),
                 torque_gains=TorqueStageGains(
                     kp=float(tau_cfg.get("kp", 40.0)),
                     kd=float(tau_cfg.get("kd", 1.5)),
                     tau_limit=float(tau_cfg.get("tau_limit", 60.0)),
+                    use_gravity_comp=bool(tau_cfg.get("use_gravity_comp", False)),
+                    gravity_scale=float(tau_cfg.get("gravity_scale", 1.0)),
                 ),
             )
         else:
             # torque_pd (backward compatible: allow flat kp/kd/tau_limit in YAML)
             gains = TorquePD(
                 kp=float(ctrl_cfg.get("kp", (ctrl_cfg.get("torque") or {}).get("kp", 40.0))),
                 kd=float(ctrl_cfg.get("kd", (ctrl_cfg.get("torque") or {}).get("kd", 1.5))),
                 tau_limit=float(ctrl_cfg.get("tau_limit", (ctrl_cfg.get("torque") or {}).get("tau_limit", 60.0))),
             )
             controller = PDPostureTorque(
                 q_ref,
                 gains=gains,
                 gravity_comp=GravityCompensation(sim.robot_id),
             )
```

---

## 3) `config/agent_tuning.yaml`（two_stage の torque ブロック拡張）

```diff
--- a/config/agent_tuning.yaml
+++ b/config/agent_tuning.yaml
@@ -1,30 +1,36 @@
 controller:
   type: two_stage
 
   warmup_seconds: 1.0
 
   # Warmup stage (POSITION_CONTROL)
   position:
     kp: 0.3
     kd: 0.1
 
   # Hold stage (TORQUE_CONTROL PD)
   torque:
     kp: 40.0
     kd: 1.5
     tau_limit: 60.0
+    # Gravity compensation in hold stage (optional)
+    use_gravity_comp: false
+    gravity_scale: 1.0
```

---

## 4) `scripts/sweep_tuning.py`（grav探索オプション＆YAML書き込み）

> ※あなたの現状 `sweep_tuning.py` が既に `--warmup` を持っている前提で、`--grav` と `--grav-scale` を追加し、two_stage の `controller.torque` に書き込みます。

```diff
--- a/scripts/sweep_tuning.py
+++ b/scripts/sweep_tuning.py
@@ -1,10 +1,12 @@
     ap.add_argument("--mode", choices=["grid", "random"], default="grid")
     ap.add_argument("--trials", type=int, default=20)
     ap.add_argument("--seed", type=int, default=0)
     ap.add_argument("--no-gui", action="store_true")
     ap.add_argument("--dry-run", action="store_true")
     ap.add_argument("--warmup", default="1.0", help="Comma-separated warmup_seconds candidates (two_stage)")
+    ap.add_argument("--grav", default="0,1", help="Comma-separated use_gravity_comp candidates: 0 or 1 (two_stage)")
+    ap.add_argument("--grav-scale", default="1.0", help="Comma-separated gravity_scale candidates (two_stage)")
 
@@ -1,20 +1,28 @@
     warmups = _mk_grid(parse_floats(args.warmup))
+    grav_flags = [int(float(x)) for x in args.grav.split(",") if x.strip()]
+    grav_scales = _mk_grid(parse_floats(args.grav_scale))
 
     planned_params: List[Dict[str, Any]] = []
 
     if args.mode == "grid":
-        all_params = list(itertools.product(kps, kds, taus, dts, settles, warmups))
+        all_params = list(itertools.product(kps, kds, taus, dts, settles, warmups, grav_flags, grav_scales))
         max_grid = max(1, args.trials)
         if len(all_params) > max_grid:
             step = max(1, len(all_params) // max_grid)
             all_params = all_params[::step][:max_grid]
-        for kp, kd, tau, dt, settle, warmup in all_params:
+        for kp, kd, tau, dt, settle, warmup, grav, gscale in all_params:
             planned_params.append(
                 {
                     "kp": kp,
                     "kd": kd,
                     "tau_limit": tau,
                     "control_dt": dt,
                     "settle_steps": settle,
                     "warmup_seconds": warmup,
+                    "use_gravity_comp": bool(int(grav)),
+                    "gravity_scale": float(gscale),
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
+                    "use_gravity_comp": bool(int(random.choice(grav_flags))),
+                    "gravity_scale": float(random.choice(grav_scales)),
                 }
             )
 
@@ -1,40 +1,54 @@
         ctrl_type = str(cfg_ctrl.get("type", "torque_pd"))
         if ctrl_type == "two_stage":
             cfg_ctrl["warmup_seconds"] = float(p.get("warmup_seconds", cfg_ctrl.get("warmup_seconds", 1.0)))
             torque_block = dict(cfg_ctrl.get("torque") or {})
             torque_block["kp"] = float(p["kp"])
             torque_block["kd"] = float(p["kd"])
             torque_block["tau_limit"] = float(p["tau_limit"])
+            torque_block["use_gravity_comp"] = bool(p.get("use_gravity_comp", torque_block.get("use_gravity_comp", False)))
+            torque_block["gravity_scale"] = float(p.get("gravity_scale", torque_block.get("gravity_scale", 1.0)))
             cfg_ctrl["torque"] = torque_block
         else:
             cfg_ctrl["kp"] = float(p["kp"])
             cfg_ctrl["kd"] = float(p["kd"])
             cfg_ctrl["tau_limit"] = float(p["tau_limit"])
 
@@ -1,40 +1,54 @@
     if str(best_cfg_ctrl.get("type", "torque_pd")) == "two_stage":
         best_cfg_ctrl["warmup_seconds"] = float(best.params.get("warmup_seconds", best_cfg_ctrl.get("warmup_seconds", 1.0)))
         torque_block = dict(best_cfg_ctrl.get("torque") or {})
         torque_block["kp"] = float(best.params["kp"])
         torque_block["kd"] = float(best.params["kd"])
         torque_block["tau_limit"] = float(best.params["tau_limit"])
+        torque_block["use_gravity_comp"] = bool(best.params.get("use_gravity_comp", torque_block.get("use_gravity_comp", False)))
+        torque_block["gravity_scale"] = float(best.params.get("gravity_scale", torque_block.get("gravity_scale", 1.0)))
         best_cfg_ctrl["torque"] = torque_block
     else:
         best_cfg_ctrl["kp"] = float(best.params["kp"])
         best_cfg_ctrl["kd"] = float(best.params["kd"])
         best_cfg_ctrl["tau_limit"] = float(best.params["tau_limit"])
```

---

**「重力補償 ON / OFF を score 基準で比較」**するための
`run_and_compare.py` の **実行コマンド例**を、いまの実装前提（two_stage・score共通化・seed有効）でまとめます。

---

## 前提チェック（1回だけ）

```bash
cd /workspace/hunter
```

* `config/agent_tuning.yaml`

  * `controller.type: two_stage`
  * `controller.torque.use_gravity_comp` を切り替える
* `compare_runs.py` が `--sort-by score` に対応済み
* `run_and_compare.py` は内部で `main_simulation.py`（standing-pd-ext）を呼ぶ想定

---

## ① 重力補償 OFF（baseline）を実行

まず **重力補償なし**のベースラインを数回回します。

```bash
# OFF に設定
yq -i '.controller.torque.use_gravity_comp = false' config/agent_tuning.yaml

# 実行（短めでもOK）
python3 scripts/run_and_compare.py \
  --duration 5 \
  --runs 3 \
  --sort-by score
```

生成物：

* `runs/standing_pd_ext_*.json`
* `meta.controller.torque.use_gravity_comp = false`
* `score` が compare 表に表示

---

## ② 重力補償 ON を実行

次に **重力補償あり**で同条件を回します。

```bash
# ON に設定
yq -i '.controller.torque.use_gravity_comp = true' config/agent_tuning.yaml

# 同じ条件で実行
python3 scripts/run_and_compare.py \
  --duration 5 \
  --runs 3 \
  --sort-by score
```

生成物：

* `meta.controller.torque.use_gravity_comp = true`
* score が baseline と直接比較可能

---

## ③ score 基準でまとめて比較

ログはすべて同じ `runs/` に入るので、**score 順で俯瞰**できます。

```bash
python3 scripts/compare_runs.py \
  --log-dir runs \
  --prefix standing_pd_ext \
  --sort-by score \
  --limit 10
```

### 見るポイント

* 上位に `use_gravity_comp=true` が並ぶか
* 同じ `seed` で survival / tilt / energy が改善しているか
* DONE 率が上がっているか

---

## ④（推奨）seed固定での純粋比較

再現性を重視する場合：

```bash
# seed固定（例：0）
yq -i '.runner.seed = 0' config/agent_tuning.yaml

# OFF
yq -i '.controller.torque.use_gravity_comp = false' config/agent_tuning.yaml
python3 scripts/run_and_compare.py --duration 5 --runs 2

# ON
yq -i '.controller.torque.use_gravity_comp = true' config/agent_tuning.yaml
python3 scripts/run_and_compare.py --duration 5 --runs 2

# 比較
python3 scripts/compare_runs.py --sort-by score
```

→ **同一初期条件で ON/OFF 差分**が見えます。

---

## ⑤（発展）gravity_scale も含めた比較

```bash
for s in 0.7 1.0 1.3; do
  yq -i ".controller.torque.gravity_scale = $s" config/agent_tuning.yaml
  python3 scripts/run_and_compare.py --duration 5 --runs 2
done

python3 scripts/compare_runs.py --sort-by score
```

---

## まとめ（判断基準）

重力補償が「効いている」状態は：

* score ↑
* survival_time ↑（または DONE 増）
* tilt_max_abs ↓
* energy_abs_tau_dq ↓

が **同時に起きる**はずです。


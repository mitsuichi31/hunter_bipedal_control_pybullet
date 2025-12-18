了解。次の1コミットとして「**ログ収集＋メトリクス算出＋JSON保存**」を入れるための一式を出します。

* `ext_runner.py` を拡張（毎制御更新でサンプル収集）
* `ext_metrics.py` を新規追加（roll/pitch/energy/foot slip など算出）
* `config/agent_tuning.yaml` を新規追加（エージェントが触るパラメータの外出し）
* `main_simulation.py` の `standing-pd-ext` で、その設定を読み込んで runner に渡す（最小変更）

以下、**unified diff** 形式でまとめます。

---

## 1) 追加: `src/ext_metrics.py`

```diff
--- /dev/null
+++ b/src/ext_metrics.py
@@ -0,0 +1,118 @@
+from __future__ import annotations
+from typing import Dict, Any, List
+import math
+
+
+def _energy_step(tau: Dict[str, float], dq: Dict[str, float], dt: float) -> float:
+    # Simple mech. power integral: sum |tau * dq| dt
+    e = 0.0
+    for j, tj in tau.items():
+        v = dq.get(j, 0.0)
+        e += abs(tj * v) * dt
+    return e
+
+
+def compute_metrics(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
+    """
+    samples: list of dicts produced by ext_runner (one per control update)
+    """
+    if not samples:
+        return {
+            "n": 0,
+            "survival_time": 0.0,
+            "tilt_max_abs": None,
+            "roll_max_abs": None,
+            "pitch_max_abs": None,
+            "base_z_min": None,
+            "energy_abs_tau_dq": None,
+            "foot_slip_left": None,
+            "foot_slip_right": None,
+        }
+
+    t0 = float(samples[0]["t"])
+    t1 = float(samples[-1]["t"])
+
+    roll_max = 0.0
+    pitch_max = 0.0
+    base_z_min = float("inf")
+
+    # foot slip: distance between first and last foot positions (simple)
+    fx0 = samples[0]["foot_pos"]
+    fx1 = samples[-1]["foot_pos"]
+    slip_left = None
+    slip_right = None
+    if "left" in fx0 and "left" in fx1:
+        a = fx0["left"]
+        b = fx1["left"]
+        slip_left = math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
+    if "right" in fx0 and "right" in fx1:
+        a = fx0["right"]
+        b = fx1["right"]
+        slip_right = math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
+
+    energy = 0.0
+    for s in samples:
+        roll = abs(float(s["rpy"][0]))
+        pitch = abs(float(s["rpy"][1]))
+        roll_max = max(roll_max, roll)
+        pitch_max = max(pitch_max, pitch)
+        base_z_min = min(base_z_min, float(s["base_pos"][2]))
+
+        # tau might be missing for position controllers
+        tau = s.get("tau", {})
+        dq = s.get("joint_vel", {})
+        dt = float(s.get("control_dt", 0.0)) or 0.0
+        if isinstance(tau, dict) and isinstance(dq, dict) and dt > 0:
+            energy += _energy_step(tau, dq, dt)
+
+    tilt_max = max(roll_max, pitch_max)
+    return {
+        "n": len(samples),
+        "survival_time": t1 - t0,
+        "tilt_max_abs": tilt_max,
+        "roll_max_abs": roll_max,
+        "pitch_max_abs": pitch_max,
+        "base_z_min": base_z_min,
+        "energy_abs_tau_dq": energy,
+        "foot_slip_left": slip_left,
+        "foot_slip_right": slip_right,
+    }
```

---

## 2) 変更: `src/ext_runner.py`（ログ収集＋JSON保存）

```diff
--- a/src/ext_runner.py
+++ b/src/ext_runner.py
@@ -1,33 +1,122 @@
 from ext_obs_adapter import adapt_obs
 from ext_safety import should_abort
 from ext_normalize import normalize_joint_commands
+from ext_metrics import compute_metrics
+
+import json
+import os
+import time as _time
+
+
+def _ensure_dir(path: str) -> None:
+    os.makedirs(path, exist_ok=True)
+
+
+def _now_tag() -> str:
+    return _time.strftime("%Y%m%d_%H%M%S")
+
+
+def _to_list3(x):
+    return [float(x[0]), float(x[1]), float(x[2])]
+
+
+def _tau_from_normalized_cmds(norm_cmds):
+    """
+    Extract tau dict when commands are torque/hybrid.
+    For position commands, returns {}.
+    """
+    tau = {}
+    for j, c in norm_cmds.items():
+        if isinstance(c, dict):
+            mode = c.get("mode", "position")
+            if mode == "torque":
+                tau[j] = float(c.get("value", 0.0))
+            elif mode == "hybrid":
+                # This is feedforward torque only, not total applied PD torque.
+                # Still useful for debugging; energy metric will be approximate.
+                tau[j] = float(c.get("torque", 0.0))
+    return tau
 
-def run(sim, controller, *, seconds: float, control_dt: float):
+
+def run(
+    sim,
+    controller,
+    *,
+    seconds: float,
+    control_dt: float,
+    settle_steps: int = 0,
+    log_dir: str = "runs",
+    run_name: str = "standing_pd_ext",
+):
     raw = sim.get_observations()
     obs = adapt_obs(raw)
     if hasattr(controller, "reset"):
         controller.reset(obs)
 
     last_control_t = obs.t
     end_t = obs.t + seconds
 
     steps = 0
     updates = 0
+    samples = []
+
+    # Optional settling period after reset (lets contacts stabilize)
+    for _ in range(max(0, int(settle_steps))):
+        sim.step()
+        steps += 1
+
+    abort_info = None
 
     while True:
         raw = sim.get_observations()
         obs = adapt_obs(raw)
 
         if obs.t >= end_t:
             break
 
         abort, reason = should_abort(obs)
         if abort:
-            return {"status": "ABORT", "t": obs.t, "reason": reason, "steps": steps, "updates": updates}
+            abort_info = {"t": float(obs.t), "reason": reason}
+            break
 
         if (obs.t - last_control_t) >= control_dt:
             joint_cmds = controller.step(obs)
-            sim.apply_hybrid_command(normalize_joint_commands(joint_cmds))
+            norm_cmds = normalize_joint_commands(joint_cmds)
+            sim.apply_hybrid_command(norm_cmds)
             last_control_t = obs.t
             updates += 1
+
+            # log one sample per control update
+            roll, pitch, yaw = obs.raw.get("rpy", (None, None, None)) if isinstance(obs.raw, dict) else (None, None, None)
+            # ext_safety already computes rpy from quat; we recompute here by importing function would be extra,
+            # so store quat + base and let downstream compute if needed.
+            # For convenience, store rpy using ext_safety's helper (import locally to avoid cycles).
+            from ext_safety import quat_to_rpy_xyzw
+            r, p, y = quat_to_rpy_xyzw(obs.base_quat_xyzw)
+
+            samples.append(
+                {
+                    "t": float(obs.t),
+                    "control_dt": float(control_dt),
+                    "base_pos": _to_list3(obs.base_pos),
+                    "base_quat_xyzw": [float(v) for v in obs.base_quat_xyzw],
+                    "rpy": [float(r), float(p), float(y)],
+                    "base_vel": _to_list3(obs.base_vel),
+                    "base_omega": _to_list3(obs.base_omega),
+                    "joint_pos": {k: float(v) for k, v in obs.joint_pos.items()},
+                    "joint_vel": {k: float(v) for k, v in obs.joint_vel.items()},
+                    "contact_forces": {k: _to_list3(v) for k, v in obs.contact_forces.items()},
+                    "foot_pos": {k: _to_list3(v) for k, v in obs.foot_pos.items()},
+                    "tau": _tau_from_normalized_cmds(norm_cmds),
+                }
+            )
 
         sim.step()
         steps += 1
 
-    return {"status": "DONE", "t": obs.t, "steps": steps, "updates": updates}
+    metrics = compute_metrics(samples)
+
+    result = {
+        "status": "ABORT" if abort_info else "DONE",
+        "t": float(obs.t),
+        "steps": steps,
+        "updates": updates,
+        "abort": abort_info,
+        "metrics": metrics,
+    }
+
+    # Save run JSON (agent reads these)
+    _ensure_dir(log_dir)
+    path = os.path.join(log_dir, f"{run_name}_{_now_tag()}.json")
+    payload = {
+        "result": result,
+        "samples": samples,  # can be large; keep for now for debugging/agent learning
+    }
+    with open(path, "w", encoding="utf-8") as f:
+        json.dump(payload, f, ensure_ascii=False, indent=2)
+
+    result["log_path"] = path
+    return result
```

これで、実行するたびに `runs/standing_pd_ext_YYYYMMDD_HHMMSS.json` が生成され、エージェントが次を自動で行えるようになります：

* 最新ログを読む
* `metrics.survival_time` を最大化するように `kp/kd/tau_limit/control_dt/settle_steps` を提案
* 再実行して比較

---

## 3) 追加: `config/agent_tuning.yaml`（エージェントが触る設定）

リポジトリに `config/` が既にあるので（あなたの説明上）、そこに追加する想定です。なければ `config/` を作ってOK。

```diff
--- /dev/null
+++ b/config/agent_tuning.yaml
@@ -0,0 +1,28 @@
+# Agent-tunable parameters for standing-pd-ext
+
+runner:
+  seconds: 10.0
+  # Controller update period. 0.01 = 100 Hz, 0.001 = 1 kHz
+  control_dt: 0.01
+  # Number of physics ticks to wait after reset before starting control
+  settle_steps: 300
+  log_dir: "runs"
+  run_name: "standing_pd_ext"
+
+safety:
+  max_roll: 0.7
+  max_pitch: 0.7
+  min_base_z: 0.12
+  max_omega: 20.0
+
+controller:
+  # For PDPostureTorque
+  kp: 40.0
+  kd: 1.5
+  tau_limit: 60.0
```

---

## 4) 変更: `src/ext_safety.py`（YAMLから閾値を渡せるように）

関数シグネチャは既にキーワード引数になってるので、**変更不要**です。
（runner側から `should_abort(obs, **safety_cfg)` と渡すだけで済む）

→ なのでここは「手順だけ」：次項で runner を呼ぶ側から渡します。

---

## 5) 変更: `main_simulation.py`（standing-pd-extでYAMLを読む）

YAMLの読み込みは、既存に `config_loader.py` があるので、そこを使うのが理想ですが、まずは最小依存で `yaml` を直接読む形にします（コンテナに PyYAML が無ければ、既存の loader に寄せます）。

### unified diff（`standing-pd-ext` の中だけ最小変更）

※あなたが追加した `run_standing_pd_ext()` がある前提です。

```diff
--- a/src/main_simulation.py
+++ b/src/main_simulation.py
@@ -210,6 +210,7 @@
 def run_standing_pd_ext(duration: float = 10.0, use_gui: bool = True):
@@
-    from ext_standing_ref import standing_q_ref
-    from ext_pd_posture_torque import PDPostureTorque
-    from ext_runner import run
+    from ext_standing_ref import standing_q_ref
+    from ext_pd_posture_torque import PDPostureTorque, TorquePD
+    from ext_runner import run
+    import yaml
 
-    controller = PDPostureTorque(standing_q_ref())
-    result = run(sim, controller, seconds=duration, control_dt=0.01)  # 100 Hz controller update
+    # Load agent-tunable config (optional)
+    cfg_path = os.path.join(script_dir, "../config/agent_tuning.yaml")
+    cfg = {}
+    try:
+        with open(cfg_path, "r", encoding="utf-8") as f:
+            cfg = yaml.safe_load(f) or {}
+    except FileNotFoundError:
+        cfg = {}
+
+    runner_cfg = (cfg.get("runner") or {})
+    ctrl_cfg = (cfg.get("controller") or {})
+
+    # Controller gains from YAML (fallback to defaults)
+    gains = TorquePD(
+        kp=float(ctrl_cfg.get("kp", 40.0)),
+        kd=float(ctrl_cfg.get("kd", 1.5)),
+        tau_limit=float(ctrl_cfg.get("tau_limit", 60.0)),
+    )
+    controller = PDPostureTorque(standing_q_ref(), gains=gains)
+
+    seconds = float(runner_cfg.get("seconds", duration))
+    control_dt = float(runner_cfg.get("control_dt", 0.01))
+    settle_steps = int(runner_cfg.get("settle_steps", 0))
+    log_dir = str(runner_cfg.get("log_dir", "runs"))
+    run_name = str(runner_cfg.get("run_name", "standing_pd_ext"))
+
+    result = run(
+        sim,
+        controller,
+        seconds=seconds,
+        control_dt=control_dt,
+        settle_steps=settle_steps,
+        log_dir=log_dir,
+        run_name=run_name,
+    )
     print(result)
```

> もしコンテナに `yaml` が入っていなくて import error が出たら、すぐ代替案を出します（既存 `config_loader.py` に寄せて読む形）。

---

## 6) 実行（Docker内）

```bash
cd /workspace/hunter/src
python3 main_simulation.py --mode standing-pd-ext --duration 10 --no-gui
```

成功すると出力に `log_path` が含まれ、`runs/....json` が生成されます。


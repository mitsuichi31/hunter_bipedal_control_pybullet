方針：既存モードは壊さず **新モード `standing-pd-ext` を追加**して外付けループで動作確認 → その後に参考コード/MPC/WBCへ差し替え可能にする

---

# 0. 前提

* 実行は Docker コンテナ内で `cd /workspace/hunter/src && python3 main_simulation.py ...`
* 環境は Gym ではなく `HunterSimulation`（`step()`は戻り値なし、観測は `get_observations()`）
* アクションは最終的に `sim.apply_hybrid_command(commands)` が受け付ける形式に揃える（これが既存の標準ルート） 

---

# 1. 追加するファイル（すべて `src/` 直下）

既存の import がフラット（`from xxx import ...`）なので、**新規も `src/` 直下に置く**のが最も事故が少ないです。

## 1-1) `src/ext_joints.py`

```python
LEG_JOINTS = [
    "leg_l1_joint","leg_l2_joint","leg_l3_joint","leg_l4_joint","leg_l5_joint",
    "leg_r1_joint","leg_r2_joint","leg_r3_joint","leg_r4_joint","leg_r5_joint",
]
```

## 1-2) `src/ext_obs_adapter.py`

（raw obs dict を、制御が扱いやすい形に正規化し、10関節の `q/dq` ベクトルも作る）

```python
from dataclasses import dataclass
from typing import Dict, Any
import numpy as np
from ext_joints import LEG_JOINTS

@dataclass
class ExtObs:
    t: float
    base_pos: np.ndarray
    base_quat_xyzw: np.ndarray
    base_vel: np.ndarray
    base_omega: np.ndarray
    joint_pos: Dict[str, float]
    joint_vel: Dict[str, float]
    q: np.ndarray
    dq: np.ndarray
    contact_forces: Dict[str, np.ndarray]
    foot_pos: Dict[str, np.ndarray]
    raw: Dict[str, Any]

def adapt_obs(raw: Dict[str, Any]) -> ExtObs:
    js = raw["joint_states"]
    joint_pos = {jn: float(pv[0]) for jn, pv in js.items()}
    joint_vel = {jn: float(pv[1]) for jn, pv in js.items()}
    q = np.array([joint_pos[j] for j in LEG_JOINTS], dtype=float)
    dq = np.array([joint_vel[j] for j in LEG_JOINTS], dtype=float)

    return ExtObs(
        t=float(raw["time"]),
        base_pos=np.asarray(raw["base_position"], float),
        base_quat_xyzw=np.asarray(raw["base_orientation"], float),
        base_vel=np.asarray(raw["base_velocity"], float),
        base_omega=np.asarray(raw["base_angular_velocity"], float),
        joint_pos=joint_pos,
        joint_vel=joint_vel,
        q=q, dq=dq,
        contact_forces={k: np.asarray(v, float) for k, v in raw["contact_forces"].items()},
        foot_pos={k: np.asarray(v, float) for k, v in raw["foot_positions"].items()},
        raw=raw,
    )
```

## 1-3) `src/ext_safety.py`

（倒れた／暴走したら止める安全ゲート）

```python
import numpy as np

def quat_to_rpy_xyzw(q):
    x,y,z,w = q
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2*(w*y - z*x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def should_abort(obs, *, max_roll=0.7, max_pitch=0.7, min_base_z=0.12, max_omega=20.0):
    roll, pitch, _ = quat_to_rpy_xyzw(obs.base_quat_xyzw)
    if abs(roll) > max_roll or abs(pitch) > max_pitch:
        return True, f"tilt too large roll={roll:.3f} pitch={pitch:.3f}"
    if float(obs.base_pos[2]) < min_base_z:
        return True, f"base too low z={obs.base_pos[2]:.3f}"
    if float(np.linalg.norm(obs.base_omega)) > max_omega:
        return True, f"omega too large |w|={np.linalg.norm(obs.base_omega):.3f}"
    return False, ""
```

## 1-4) `src/ext_normalize.py`（重要）

あなたの `HunterSimulation.apply_hybrid_command()` の実装に **完全一致**する形に正規化します 

```python
from typing import Dict, Any, Union

HybridValue = Union[float, int, Dict[str, Any]]
JointCommands = Dict[str, HybridValue]

def normalize_joint_commands(cmds: JointCommands) -> JointCommands:
    out: JointCommands = {}

    for j, c in cmds.items():
        if isinstance(c, (float, int)):
            out[j] = float(c)
            continue

        if not isinstance(c, dict):
            raise TypeError(f"Unsupported command type for {j}: {type(c)}")

        mode = c.get("mode", "position")

        if mode == "torque":
            if "value" in c:
                out[j] = {"mode": "torque", "value": float(c["value"])}
            elif "torque" in c:
                out[j] = {"mode": "torque", "value": float(c["torque"])}
            else:
                raise KeyError(f"torque mode needs 'value' or 'torque' for {j}")

        elif mode == "hybrid":
            if "position" not in c and "value" not in c:
                raise KeyError(f"hybrid mode needs 'position' or 'value' for {j}")
            out[j] = {
                "mode": "hybrid",
                "position": float(c.get("position", c.get("value"))),
                "velocity": float(c.get("velocity", 0.0)),
                "kp": float(c.get("kp", 0.0)),
                "kd": float(c.get("kd", 0.0)),
                "torque": float(c.get("torque", 0.0)),
            }

        else:  # position
            if "value" in c:
                target = c["value"]
            elif "position" in c:
                target = c["position"]
            else:
                raise KeyError(f"position mode needs 'value' or 'position' for {j}")

            out[j] = {"mode": "position", "value": float(target)}
            if "kp" in c: out[j]["kp"] = float(c["kp"])
            if "kd" in c: out[j]["kd"] = float(c["kd"])
            if "velocity" in c: out[j]["velocity"] = float(c["velocity"])

    return out
```

## 1-5) `src/ext_runner.py`

（tick→obs→controller→normalize→apply→tick）

```python
from ext_obs_adapter import adapt_obs
from ext_safety import should_abort
from ext_normalize import normalize_joint_commands

def run(sim, controller, *, seconds: float, control_dt: float):
    raw = sim.get_observations()
    obs = adapt_obs(raw)
    if hasattr(controller, "reset"):
        controller.reset(obs)

    last_control_t = obs.t
    end_t = obs.t + seconds

    steps = 0
    updates = 0

    while True:
        raw = sim.get_observations()
        obs = adapt_obs(raw)

        if obs.t >= end_t:
            break

        abort, reason = should_abort(obs)
        if abort:
            return {"status": "ABORT", "t": obs.t, "reason": reason, "steps": steps, "updates": updates}

        if (obs.t - last_control_t) >= control_dt:
            joint_cmds = controller.step(obs)
            sim.apply_hybrid_command(normalize_joint_commands(joint_cmds))
            last_control_t = obs.t
            updates += 1

        sim.step()
        steps += 1

    return {"status": "DONE", "t": obs.t, "steps": steps, "updates": updates}
```

## 1-6) `src/ext_standing_ref.py`

（`robot_constants.STANDING_CONFIG` → (10,) ベクトル）

```python
import numpy as np
from robot_constants import STANDING_CONFIG
from ext_joints import LEG_JOINTS

def standing_q_ref() -> np.ndarray:
    return np.array([STANDING_CONFIG[j] for j in LEG_JOINTS], dtype=float)
```

## 1-7) `src/ext_pd_posture_torque.py`

（立位スモーク用：最も安全な torque-only）

```python
from dataclasses import dataclass
import numpy as np
from ext_joints import LEG_JOINTS

@dataclass
class TorquePD:
    kp: float = 40.0
    kd: float = 1.5
    tau_limit: float = 60.0

class PDPostureTorque:
    def __init__(self, q_ref: np.ndarray, gains: TorquePD = TorquePD()):
        assert q_ref.shape == (10,)
        self.q_ref = q_ref.astype(float)
        self.g = gains

    def reset(self, obs):
        pass

    def step(self, obs):
        tau = self.g.kp * (self.q_ref - obs.q) - self.g.kd * obs.dq
        tau = np.clip(tau, -self.g.tau_limit, self.g.tau_limit)
        return {j: {"mode": "torque", "value": float(t)} for j, t in zip(LEG_JOINTS, tau)}
```

---

# 2. `src/main_simulation.py` の変更（最小・新モード追加のみ）

既存モードは一切いじらず、**新しいモード `standing-pd-ext` を追加**します。

## 2-1) argparse の choices に追加

（`choices=["standing", "standing-mpc", "wbc", "walking", ...]` の場所に追加）

```diff
- choices=["standing", "standing-mpc", "wbc", "walking"],
+ choices=["standing", "standing-mpc", "wbc", "walking", "standing-pd-ext"],
```

## 2-2) 分岐を追加（初期化は既存standing-mpcの流儀で安全側）

`if args.mode == ...` の分岐に、以下を追加します。

```python
elif args.mode == "standing-pd-ext":
    # 既存と同じ手順で config を読み、limits を構築（既存関数を再利用）
    task_config = load_task_config()
    physics_params, command_limits = _build_physics_and_limits(task_config)

    # URDF パスも既存の流儀で
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

    # シミュ初期化（dt=0.001, stable contacts ON）
    sim = HunterSimulation(
        urdf_path=urdf_path,
        dt=0.001,
        use_gui=use_gui,
        physics_params=physics_params,
        command_limits=command_limits,
    )
    sim.connect(enable_stable_contacts=True)
    sim.load_robot(start_position=[0, 0, BASE_HEIGHT])

    # 立位姿勢へ reset（existing standing/wbc と同様）
    sim.reset_robot(position=[0, 0, BASE_HEIGHT], joint_positions=STANDING_CONFIG)

    from ext_standing_ref import standing_q_ref
    from ext_pd_posture_torque import PDPostureTorque
    from ext_runner import run

    controller = PDPostureTorque(standing_q_ref())
    result = run(sim, controller, seconds=args.duration, control_dt=0.01)  # 100Hz
    print(result)

    sim.disconnect()
```

---

# 3. Docker コンテナでの実行（ビルド不要）

Pythonファイル追加だけなので、基本は **build不要**で即反映です（ボリュームマウント運用）。
実行は DOCKER.md の流れのままでOK。

コンテナ内：

```bash
cd /workspace/hunter/src
python3 main_simulation.py --mode standing-pd-ext --duration 10 --no-gui
```

GUIを使うなら（DOCKER.md推奨の環境変数）：

```bash
cd /workspace/hunter/src
QT_X11_NO_MITSHM=1 PYBULLET_USE_OPENGL2=1 \
python3 main_simulation.py --mode standing-pd-ext --duration 10
```

---

# 4. 動作確認のチェックポイント

* `KeyError` などコマンド形式エラーが出ない
  → `ext_normalize.py` が `apply_hybrid_command()` に完全一致してるので出ないはず 
* 10秒間 `DONE` で終了、または倒れて `ABORT`（reasonが出る）
* 関節名 missing が出ない（10関節が `joint_states` に入っていること）

---

# 5. 次の拡張（参考コード／MPC／WBCへ差し替える）

この手順で “外付けループ” が動いたら、AIエージェント運用的にはここが本番です。

* `controller = PDPostureTorque(...)` を
* `controller = RefAdapter(参考コントローラ or MPCWBCController, map_in, map_out)` に置き換える

その時も `ext_runner.py` と `normalize→apply_hybrid_command` は固定なので、
**「制御器だけ差し替えて比較実験」**が安全に回ります。

---

以上が、ここまでの検討を**あなたの既存コード＋Docker環境**へ適用する完全手順です。


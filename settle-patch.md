状況、かなりはっきりしました。

* survival が **0.33〜0.70s**で全落ち
* warmup_seconds は **1.5s**（＝ warmup が終わる前に落ちてる）
* さらに `settle_steps=300`（＝0.3s）を入れている

この条件だと、**落ちているのはほぼ “warmup以前/settle中”**です。
そしてここでよく起きる致命的パターンが：

> **settle_steps の間に一度もモータ目標（position/hybrid/torque）をセットしていない**
> → 0.3秒 “脱力” で物理だけ進む → 倒れる → warmupゲインをいくら上げても意味が薄い

あなたの観測（pos sweepで `tau` が空扱い＝positionのみで、hold/blendまで到達してない）とも整合します。

---

# 次ステップ：settle 中に「初回コマンドを必ず当てる」パッチ（unified diff）

対象：`src/ext_runner.py`
やること：`settle_steps` に入る前に **1回だけでも** controller の warmup(position) コマンドを生成して `sim.apply_hybrid_command()` でセットしておく（PyBulletは最後にセットしたモータ制御が維持されるので、1回で効きます）。

```diff
--- a/src/ext_runner.py
+++ b/src/ext_runner.py
@@ -1,6 +1,7 @@
 from ext_obs_adapter import adapt_obs
 from ext_safety import should_abort
 from ext_normalize import normalize_joint_commands
 from ext_metrics import compute_metrics
+from typing import Optional, Dict, Any
 
 import json
 import os
 import time as _time
@@ -80,6 +81,34 @@ def run(
     samples = []
     abort_info = None
 
+    # --- IMPORTANT ---
+    # If we "settle" the physics before sending ANY motor command, the robot can fall
+    # during settle_steps (e.g., 300 ticks = 0.3s). That makes warmup tuning pointless.
+    # So: apply an initial command once before settle.
+    try:
+        raw_obs0 = sim.get_observations()
+        obs0 = adapt_obs(raw_obs0)
+        # Ensure controller has time origin
+        if hasattr(controller, "reset"):
+            controller.reset(obs0)
+        joint_commands0 = controller.step(obs0)
+        joint_commands0 = normalize_joint_commands(joint_commands0)
+        sim.apply_hybrid_command(joint_commands0)
+        # Record that we applied initial command
+        samples.append({"t": float(obs0.t), "status": "INIT_CMD"})
+    except Exception as e:
+        # If something goes wrong here, we still continue; main loop will try again.
+        samples.append({"t": 0.0, "status": "INIT_CMD_FAIL", "error": str(e)[:120]})
+
     # settle
     for _ in range(int(settle_steps or 0)):
         sim.step()
 
     # main loop
     last_control = -1e9
     updates = 0
@@ -101,6 +130,8 @@ def run(
         raw_obs = sim.get_observations()
         sim_time = float(raw_obs.get("time", 0.0))
 
         if (sim_time - last_control) >= control_dt:
             last_control = sim_time
             updates += 1
 
             obs = adapt_obs(raw_obs)
             joint_commands = controller.step(obs)
             joint_commands = normalize_joint_commands(joint_commands)
             sim.apply_hybrid_command(joint_commands)
```

---

## これを入れた後の「再確認 sweep」

まずは “settle が原因だったか” を切り分けるために、同じ pos sweep を **settle=300 のまま**再実行してください。

```bash
cd /workspace/hunter
python3 scripts/sweep_tuning.py \
  --mode grid --grid-sample spread \
  --trials 18 --repeats 1 --no-gui \
  --control-dt 0.01 \
  --settle 300 \
  --warmup 1.5 \
  --blend 0.4 \
  --grav 1 --grav-scale 1.0 \
  --kp 20 --kd 4.5 --kd-blend 3.0 --tau 150 \
  --pos-kp 10,30,60,120 \
  --pos-kd 0.5,1.0,2.0,4.0
```

期待する変化：

* survival が **最低でも 0.3s 分は伸びやすい**（settle中に倒れなくなる）
* もし warmup が効けば 1.5s 以上まで伸びて blend/hold に入るケースが出る
* その場合、`clip_mean` や `energy` も（blend/hold に入った trial では）出始めます

---

## 補足：今回 `clip_mean/energy` が “- / 0.0” になった理由

これは異常ではなく、**position-only（mode="position"）の間は tau をログしてない/算出してない**からです。
ただし、上のパッチで “まず立って warmup→blend→hold に入れる” ようになれば、以降は通常どおり指標が埋まります。

---

このパッチを当てた結果（survival がどう動いたか、INIT_CMD が入ったか）を教えてくれれば、次は **pos_kp を 120→300→600 に上げるべきか**、それとも **初期姿勢/BASE_HEIGHT/tilt閾値**側を疑うべきかを、かなり確実に分岐できます。

ここまでのまとめ、完全に「次に何をやるべきか」が決まりました。

* **接触（ct_any）がほぼゼロ（~0.0167）**
* **足接触が起きる瞬間はある（FOOT_CONTACT が出る）**
* **foot_dy_min がほぼ 0（足幅が潰れて左右が寄る/交差）**
* **BASE_HEIGHT の許容域が極端に狭い（0.679 だけ良い）**

これ、いまの主因はコントローラや安全閾値より **「初期条件（足幅/接地ジオメトリ）と接触成立条件が崩れている」**です。

次ステップは **“足幅（hip roll = l1/r1）を sweep して foot_dy を確保し、ct_any を増やす”** です。
crouch や base_h は触ってもよくなりますが、今のログが示してる最大の異常は **foot_dy_min ≈ 0** なので、ここをまず直撃します。

---

# 次ステップ：足幅（hip roll）を YAML/Sweep で駆動して探索する

## 狙い

* `leg_l1_joint` / `leg_r1_joint`（hip roll）を stance ターゲットに上乗せして、**初期の足幅を確実に確保**
* その結果として

  * `foot_dy_min` が 0 から十分大きくなる
  * `ct_any` が増える（接触が成立しやすい）
  * roll の発散が抑えられる
  * 1秒壁を越える可能性が出る

---

# パッチ一式（unified diff）

## 1) `src/ext_standing_ref.py`：hip_roll_delta を導入（左右に±で足幅を作る）

```diff
--- a/src/ext_standing_ref.py
+++ b/src/ext_standing_ref.py
@@ -1,6 +1,6 @@
 from __future__ import annotations
-from typing import Dict, Optional
+from typing import Dict, Optional
 
 from robot_constants import STANDING_CONFIG, get_stance_config
 
@@
 def stance_q_ref(
     stance: str,
     crouch_knee: Optional[float] = None,
     crouch_ankle: Optional[float] = None,
+    hip_roll_delta: Optional[float] = None,
 ) -> Dict[str, float]:
@@
     cfg = dict(get_stance_config(stance))
@@
     if s.startswith("crouch") and (crouch_knee is not None or crouch_ankle is not None):
         # existing crouch override logic...
         pass
+
+    # Widen / narrow stance by hip roll. Apply symmetric ±delta on l1/r1.
+    if hip_roll_delta is not None:
+        try:
+            d = float(hip_roll_delta)
+            # convention: l1 negative, r1 positive widens
+            if "leg_l1_joint" in cfg:
+                cfg["leg_l1_joint"] = float(cfg["leg_l1_joint"]) - abs(d)
+            if "leg_r1_joint" in cfg:
+                cfg["leg_r1_joint"] = float(cfg["leg_r1_joint"]) + abs(d)
+        except Exception:
+            pass
     return cfg
```

---

## 2) `config/agent_tuning.yaml`：controller.hip_roll_delta を追加

```diff
--- a/config/agent_tuning.yaml
+++ b/config/agent_tuning.yaml
@@ -6,6 +6,10 @@ runner:
 controller:
   type: two_stage
   stance: standing
+  # Extra widening on top of stance template (rad). 0 keeps current behavior.
+  # l1 -= abs(delta), r1 += abs(delta)
+  hip_roll_delta: 0.0
   crouch_knee: 0.6
   crouch_ankle: 0.3
```

---

## 3) `src/main_simulation.py`：hip_roll_delta を読み、stance_q_ref に渡す（metaにも保存）

```diff
--- a/src/main_simulation.py
+++ b/src/main_simulation.py
@@ -1268,6 +1268,7 @@
     stance_name = ctrl_cfg.get("stance", "standing")
     crouch_knee = ctrl_cfg.get("crouch_knee", None)
     crouch_ankle = ctrl_cfg.get("crouch_ankle", None)
+    hip_roll_delta = ctrl_cfg.get("hip_roll_delta", None)
     target_positions = stance_q_ref(
         stance_name,
         crouch_knee=crouch_knee,
         crouch_ankle=crouch_ankle,
+        hip_roll_delta=hip_roll_delta,
     )
@@
     if isinstance(run_meta, dict):
         run_meta.setdefault("controller", {})
         if isinstance(run_meta["controller"], dict):
             run_meta["controller"]["stance"] = str(stance_name)
+            if hip_roll_delta is not None:
+                try:
+                    run_meta["controller"]["hip_roll_delta"] = float(hip_roll_delta)
+                except Exception:
+                    pass
             if crouch_knee is not None:
                 run_meta["controller"]["crouch_knee"] = float(crouch_knee)
             if crouch_ankle is not None:
                 run_meta["controller"]["crouch_ankle"] = float(crouch_ankle)
```

---

## 4) `scripts/sweep_tuning.py`：`--hip-roll-delta` を追加して trial/YAML に反映

```diff
--- a/scripts/sweep_tuning.py
+++ b/scripts/sweep_tuning.py
@@ -154,6 +154,7 @@
     ap.add_argument("--max-roll", default=None, help="Comma-separated safety max_roll (rad) overrides")
     ap.add_argument("--max-pitch", default=None, help="Comma-separated safety max_pitch (rad) overrides")
     ap.add_argument("--base-height", default=None, help="Comma-separated runner base_height overrides (m)")
+    ap.add_argument("--hip-roll-delta", default=None, help="Comma-separated controller hip_roll_delta (rad)")
@@ -205,6 +206,7 @@
     max_pitches = _mk_grid(parse_floats(args.max_pitch)) if args.max_pitch is not None else None
     base_heights = _mk_grid(parse_floats(args.base_height)) if args.base_height is not None else None
+    hip_deltas = _mk_grid(parse_floats(args.hip_roll_delta)) if args.hip_roll_delta is not None else None
@@ -221,6 +223,7 @@
         _mp = max_pitches if max_pitches is not None else [None]
         _bh = base_heights if base_heights is not None else [None]
+        _hd = hip_deltas if hip_deltas is not None else [None]
         all_params = list(
             itertools.product(
@@
-                _pos_kps, _pos_kds, stances, _ck, _ca, _mr, _mp, _bh
+                _pos_kps, _pos_kds, stances, _ck, _ca, _mr, _mp, _bh, _hd
             )
         )
@@ -230,7 +233,7 @@
         if len(all_params) > max_grid:
             all_params = _downsample_grid(all_params, max_grid)
-        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend, pos_kp, pos_kd, stance, ck, ca, mr, mp, bh in all_params:
+        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend, pos_kp, pos_kd, stance, ck, ca, mr, mp, bh, hd in all_params:
             planned_params.append(
                 {
@@
                     "base_height": float(bh) if bh is not None else None,
+                    "hip_roll_delta": float(hd) if hd is not None else None,
                 }
             )
@@ -275,6 +278,7 @@
         if ctrl_type == "two_stage":
             if p.get("stance", None) is not None:
                 cfg_ctrl["stance"] = str(p["stance"])
+            if p.get("hip_roll_delta", None) is not None:
+                cfg_ctrl["hip_roll_delta"] = float(p["hip_roll_delta"])
             if p.get("crouch_knee", None) is not None:
                 cfg_ctrl["crouch_knee"] = float(p["crouch_knee"])
             if p.get("crouch_ankle", None) is not None:
                 cfg_ctrl["crouch_ankle"] = float(p["crouch_ankle"])
```

---

## 5) `scripts/compare_runs.py`：hip_roll_delta 列を追加（任意だが便利）

```diff
--- a/scripts/compare_runs.py
+++ b/scripts/compare_runs.py
@@ -18,6 +18,7 @@ class RunSummary:
     crouch_knee: Optional[float]
     crouch_ankle: Optional[float]
     base_height: Optional[float]
+    hip_roll_delta: Optional[float]
@@
 def _summarize_run(path: str) -> RunSummary:
@@
     base_h = _safe_get(meta, ["runner", "base_height"], default=None)
+    hip_d = _safe_get(meta, ["controller", "hip_roll_delta"], default=None)
@@
+    try:
+        hip_d = float(hip_d) if hip_d is not None else None
+    except Exception:
+        hip_d = None
@@
     return RunSummary(
@@
         base_height=base_h,
+        hip_roll_delta=hip_d,
@@
 def _print_table(...):
     headers = [
         "base_h",
+        "hip_d",
         "stance",
         "cknee",
         "cankle",
@@
     rows.append([
         _fmt(r.base_height, 3),
+        _fmt(r.hip_roll_delta, 3),
         (r.stance or "-"),
         _fmt(r.crouch_knee, 3),
         _fmt(r.crouch_ankle, 3),
```

---

# 次に回すコマンド（最小・確実に効くやつ）

ポイントは **base_height は“最良の 0.679 に固定”**して、hip_roll_delta だけ振ることです。
（高さは超シビアなので、ここを動かさない）

```bash
cd /workspace/hunter

python3 scripts/sweep_tuning.py \
  --mode grid --grid-sample spread \
  --trials 10 --repeats 1 --no-gui --duration 3 \
  --control-dt 0.01 --settle 300 \
  --warmup 999 --blend 0.0 --grav 0 \
  --base-height 0.679 \
  --stance crouch_pos --crouch-knee 0.70 --crouch-ankle 0.30 \
  --pos-kp 120 --pos-kd 1.0 \
  --hip-roll-delta 0.00,0.03,0.06,0.09,0.12
```

見どころはこれだけ：

* `foot_dy_min` が **0 に張り付かなくなるか**
* `ct_any` が **0.02→0.2以上**に上がるか
* survival が 1秒を越えるか

確認：

```bash
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --limit 20
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by slip --limit 20
python3 scripts/analyze_abort_reasons.py --summary runs/sweep_summary.json --limit 12 --min-trials 1
```

---

# なぜこれが次か（短く）

いま一番壊れてるのは **foot_dy が潰れる**ことで、これは “静止姿勢の設計（足幅）” の問題。
接触が希薄（ct_any低い）なのも、「地面に乗る面積/姿勢」が崩れているのが原因になりやすいです。

---

これで `hip_roll_delta` を増やしたときに `foot_dy_min` と `ct_any` が上がるなら、次は

* stance テンプレ側の `leg_l1/r1` 自体を更新（デフォルトを広げる）
* その状態で two_stage の hold（warmupを短くして）へ戻す
  に進めます。

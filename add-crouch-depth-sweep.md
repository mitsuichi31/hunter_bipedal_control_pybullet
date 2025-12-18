いい進捗です。ここまでで **「直脚が不利」→「crouchで改善」**が定量で出たので、次はご所見どおり

1. **crouch の曲げ量（膝/足首）を sweep**（まずはこれが本命）
2. その後に **tilt閾値を観測目的で一時的に緩める**（“回復余地”があるかの切り分け）

の順が最短です。

以下に **(1) crouch depth sweep 対応**のパッチ一式（unified diff）を出します。
（既に `src/ext_standing_ref.py` を導入している前提で、そこを拡張します）

---

# パッチ一式：crouch_knee / crouch_ankle を YAML/Sweep で駆動

## 1) `src/ext_standing_ref.py`（crouch量を引数で指定できるように）

```diff
--- a/src/ext_standing_ref.py
+++ b/src/ext_standing_ref.py
@@ -1,10 +1,14 @@
 from __future__ import annotations
-from typing import Dict
+from typing import Dict, Optional
 
-from robot_constants import STANDING_CONFIG, get_stance_config
+from robot_constants import STANDING_CONFIG, get_stance_config
 
 def standing_q_ref() -> Dict[str, float]:
     return dict(STANDING_CONFIG)
 
-def stance_q_ref(stance: str) -> Dict[str, float]:
-    return dict(get_stance_config(stance))
+def stance_q_ref(
+    stance: str,
+    crouch_knee: Optional[float] = None,
+    crouch_ankle: Optional[float] = None,
+) -> Dict[str, float]:
+    """
+    Return joint position targets for a given stance.
+    If stance is crouch_* and crouch_knee/ankle are provided, override those magnitudes.
+    """
+    cfg = dict(get_stance_config(stance))
+    s = str(stance)
+    if s.startswith("crouch") and (crouch_knee is not None or crouch_ankle is not None):
+        # Joint mapping follows robot_constants crouch templates
+        if crouch_knee is not None:
+            # l3/r3 and l4/r4 are knee-like joints in your crouch templates
+            # Keep sign for l3/r3 per stance; l4/r4 must remain positive due to limits.
+            k = float(crouch_knee)
+            if s == "crouch_neg":
+                cfg["leg_l3_joint"] = -abs(k)
+                cfg["leg_r3_joint"] = -abs(k)
+                cfg["leg_l4_joint"] = +abs(k)
+                cfg["leg_r4_joint"] = +abs(k)
+            else:
+                cfg["leg_l3_joint"] = +abs(k)
+                cfg["leg_r3_joint"] = +abs(k)
+                cfg["leg_l4_joint"] = +abs(k)
+                cfg["leg_r4_joint"] = +abs(k)
+        if crouch_ankle is not None:
+            a = float(crouch_ankle)
+            if s == "crouch_neg":
+                cfg["leg_l5_joint"] = +abs(a)
+                cfg["leg_r5_joint"] = +abs(a)
+            else:
+                cfg["leg_l5_joint"] = -abs(a)
+                cfg["leg_r5_joint"] = -abs(a)
+    return cfg
```

---

## 2) `config/agent_tuning.yaml`（crouchパラメータを追加）

```diff
--- a/config/agent_tuning.yaml
+++ b/config/agent_tuning.yaml
@@ -1,10 +1,18 @@
 controller:
   type: two_stage
   stance: standing
+  # Used only when stance is crouch_pos / crouch_neg
+  # (standingでは無視されます)
+  crouch_knee: 0.6
+  crouch_ankle: 0.3
 
   warmup_seconds: 1.0
   blend_seconds: 0.2
   position:
     kp: 30.0
     kd: 1.0
   torque:
     kp: 40.0
     kd: 1.5
     tau_limit: 60.0
```

---

## 3) `src/main_simulation.py`（stance_q_ref に crouch量を渡す）

> すでに `controller.stance` を読んで reset姿勢と q_ref を切替済み、とのことなので
> “q_ref生成”の箇所だけ **crouch_knee/ankle** を渡します。

```diff
--- a/src/main_simulation.py
+++ b/src/main_simulation.py
@@ -1,5 +1,6 @@
 # ... existing imports ...
+from ext_standing_ref import stance_q_ref
 
@@ -XXXX,10 +XXXX,18 @@
-    stance_name = ctrl_cfg.get("stance", "standing")
-    target_positions = stance_q_ref(stance_name)
+    stance_name = ctrl_cfg.get("stance", "standing")
+    crouch_knee = ctrl_cfg.get("crouch_knee", None)
+    crouch_ankle = ctrl_cfg.get("crouch_ankle", None)
+    target_positions = stance_q_ref(
+        stance_name,
+        crouch_knee=crouch_knee,
+        crouch_ankle=crouch_ankle,
+    )
 
     sim.reset_robot(position=[0, 0, BASE_HEIGHT], joint_positions=target_positions)
 
     # meta（既にrun_metaにstanceを入れているなら、それに加えて保存）
     if isinstance(run_meta, dict):
         run_meta.setdefault("controller", {})
         if isinstance(run_meta["controller"], dict):
             run_meta["controller"]["stance"] = str(stance_name)
+            if crouch_knee is not None:
+                run_meta["controller"]["crouch_knee"] = float(crouch_knee)
+            if crouch_ankle is not None:
+                run_meta["controller"]["crouch_ankle"] = float(crouch_ankle)
```

---

## 4) `scripts/sweep_tuning.py`（`--crouch-knee`, `--crouch-ankle` を追加）

```diff
--- a/scripts/sweep_tuning.py
+++ b/scripts/sweep_tuning.py
@@ -145,6 +145,8 @@
     ap.add_argument("--pos-kp", default=None, help="Comma-separated warmup position kp candidates (two_stage)")
     ap.add_argument("--pos-kd", default=None, help="Comma-separated warmup position kd candidates (two_stage)")
     ap.add_argument("--stance", default="standing", help="Comma-separated stance candidates: standing,crouch_pos,crouch_neg")
+    ap.add_argument("--crouch-knee", default=None, help="Comma-separated crouch knee magnitudes (used when stance is crouch_*)")
+    ap.add_argument("--crouch-ankle", default=None, help="Comma-separated crouch ankle magnitudes (used when stance is crouch_*)")
     ap.add_argument("--grid-sample", default="spread", choices=["spread", "random"], help="How to downsample grid when --trials < full grid size")
 
@@ -191,6 +193,8 @@
     pos_kps = _mk_grid(parse_floats(args.pos_kp)) if args.pos_kp is not None else None
     pos_kds = _mk_grid(parse_floats(args.pos_kd)) if args.pos_kd is not None else None
     stances = [s.strip() for s in str(args.stance).split(",") if s.strip()]
+    crouch_knees = _mk_grid(parse_floats(args.crouch_knee)) if args.crouch_knee is not None else None
+    crouch_ankles = _mk_grid(parse_floats(args.crouch_ankle)) if args.crouch_ankle is not None else None
 
@@ -207,7 +211,9 @@
         _pos_kps = pos_kps if pos_kps is not None else [None]
         _pos_kds = pos_kds if pos_kds is not None else [None]
+        _ck = crouch_knees if crouch_knees is not None else [None]
+        _ca = crouch_ankles if crouch_ankles is not None else [None]
         all_params = list(
             itertools.product(
-                kps, kds, taus, dts, settles, warmups, blends, grav_flags, grav_scales, kd_blends, _pos_kps, _pos_kds, stances
+                kps, kds, taus, dts, settles, warmups, blends, grav_flags, grav_scales, kd_blends, _pos_kps, _pos_kds, stances, _ck, _ca
             )
         )
@@ -215,7 +221,7 @@
         if len(all_params) > max_grid:
             all_params = _downsample_grid(all_params, max_grid)
-        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend, pos_kp, pos_kd, stance in all_params:
+        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend, pos_kp, pos_kd, stance, ck, ca in all_params:
             planned_params.append(
                 {
@@ -228,6 +234,8 @@
                     "pos_kp": float(pos_kp) if pos_kp is not None else None,
                     "pos_kd": float(pos_kd) if pos_kd is not None else None,
                     "stance": str(stance),
+                    "crouch_knee": float(ck) if ck is not None else None,
+                    "crouch_ankle": float(ca) if ca is not None else None,
                 }
             )
@@ -247,6 +255,8 @@
                     "pos_kp": float(random.choice(pos_kps)) if pos_kps is not None else None,
                     "pos_kd": float(random.choice(pos_kds)) if pos_kds is not None else None,
                     "stance": str(random.choice(stances)),
+                    "crouch_knee": float(random.choice(crouch_knees)) if crouch_knees is not None else None,
+                    "crouch_ankle": float(random.choice(crouch_ankles)) if crouch_ankles is not None else None,
                 }
             )
 
@@ -274,6 +284,10 @@
         if ctrl_type == "two_stage":
             if p.get("stance", None) is not None:
                 cfg_ctrl["stance"] = str(p["stance"])
+            if p.get("crouch_knee", None) is not None:
+                cfg_ctrl["crouch_knee"] = float(p["crouch_knee"])
+            if p.get("crouch_ankle", None) is not None:
+                cfg_ctrl["crouch_ankle"] = float(p["crouch_ankle"])
             cfg_ctrl["warmup_seconds"] = float(p.get("warmup_seconds", cfg_ctrl.get("warmup_seconds", 1.0)))
             cfg_ctrl["blend_seconds"] = float(p.get("blend_seconds", cfg_ctrl.get("blend_seconds", 0.0)))
@@ -462,6 +476,10 @@
     if str(best_cfg_ctrl.get("type", "torque_pd")) == "two_stage":
         if best.params.get("stance", None) is not None:
             best_cfg_ctrl["stance"] = str(best.params["stance"])
+        if best.params.get("crouch_knee", None) is not None:
+            best_cfg_ctrl["crouch_knee"] = float(best.params["crouch_knee"])
+        if best.params.get("crouch_ankle", None) is not None:
+            best_cfg_ctrl["crouch_ankle"] = float(best.params["crouch_ankle"])
         best_cfg_ctrl["warmup_seconds"] = float(best.params.get("warmup_seconds", best_cfg_ctrl.get("warmup_seconds", 1.0)))
         best_cfg_ctrl["blend_seconds"] = float(best.params.get("blend_seconds", best_cfg_ctrl.get("blend_seconds", 0.0)))
```

---

## 5) `scripts/compare_runs.py`（crouch量の列を追加）

```diff
--- a/scripts/compare_runs.py
+++ b/scripts/compare_runs.py
@@ -18,6 +18,8 @@
 class RunSummary:
@@
     stance: Optional[str]
+    crouch_knee: Optional[float]
+    crouch_ankle: Optional[float]
@@
 def _summarize_run(path: str) -> RunSummary:
@@
     stance = _safe_get(meta, ["controller", "stance"], default=None)
+    crouch_knee = _safe_get(meta, ["controller", "crouch_knee"], default=None)
+    crouch_ankle = _safe_get(meta, ["controller", "crouch_ankle"], default=None)
@@
     try:
         stance = str(stance) if stance is not None else None
     except Exception:
         stance = None
+    try:
+        crouch_knee = float(crouch_knee) if crouch_knee is not None else None
+    except Exception:
+        crouch_knee = None
+    try:
+        crouch_ankle = float(crouch_ankle) if crouch_ankle is not None else None
+    except Exception:
+        crouch_ankle = None
 
     return RunSummary(
@@
         stance=stance,
+        crouch_knee=crouch_knee,
+        crouch_ankle=crouch_ankle,
@@
 def _print_table(...):
     headers = [
@@
         "stance",
+        "cknee",
+        "cankle",
         "pos_kp",
         "pos_kd",
@@
     rows.append([
         (r.stance or "-"),
+        _fmt(r.crouch_knee, 3),
+        _fmt(r.crouch_ankle, 3),
         _fmt(r.pos_kp, 2),
         _fmt(r.pos_kd, 2),
```

---

# 次に回すコマンド例（“1秒壁を割る”ための crouch depth sweep）

まずは position-only のまま、crouch を **浅め/中/深め**で振ります。

```bash
cd /workspace/hunter

python3 scripts/sweep_tuning.py \
  --mode grid --grid-sample spread \
  --trials 24 --repeats 1 --no-gui \
  --control-dt 0.01 --settle 300 \
  --warmup 999 --blend 0.0 --grav 0 \
  --stance crouch_pos,crouch_neg \
  --crouch-knee 0.3,0.45,0.6,0.75 \
  --crouch-ankle 0.15,0.25,0.35 \
  --pos-kp 120,240 \
  --pos-kd 1.0,2.0

python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --limit 20
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by qerr --limit 20
```

---

# （次の次）tilt閾値を緩める“観測用”オプション

これは **回復する余地があるか**を見るために一時的にやるのが有効です。
必要なら、`scripts/sweep_tuning.py` に `--max-roll`, `--max-pitch` を足して YAML の safety を上書きするパッチも同じ形式で出します（ただしスコア比較の公平性のため、この実験は“切り分け専用”推奨）。

---

この crouch depth sweep を回したら、上位（score順 or survival順）で

* stance
* crouch_knee / crouch_ankle
* base_z_min（または base_z_min_sampled）
* roll/pitch_max

を見れば、**「深くしゃがむほど良い」か「浅い方が安定」か**がすぐ決まります。

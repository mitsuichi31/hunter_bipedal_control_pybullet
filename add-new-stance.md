了解です。ここまでの切り分けで **q_err_rms が極小＝「目標姿勢には追従できている」**が確定したので、次はほぼ **「q_ref（直脚STANDING_CONFIG）がそもそも不安定」**を潰すフェーズです。

次ステップは **“crouch（膝曲げ・腰下げ）姿勢” を導入して stance を sweep 可能にする**のが最短です。
（直脚は重心が高く、接触の微小な滑り・足裏角度ズレが即 tilt に繋がりやすい）

以下に **新規 stance 追加＋main_simulationで選択＋sweep対応＋compare表示**の **パッチ一式（unified diff）**を出します。

---

# 1) `src/robot_constants.py`（stance追加：standing/crouch_pos/crouch_neg）

> ※符号が不明なので、膝曲げ方向を **pos/neg の2系統**用意して sweep で当てに行きます。

```diff
--- a/src/robot_constants.py
+++ b/src/robot_constants.py
@@ -1,6 +1,7 @@
 from __future__ import annotations
 from typing import Dict, List
 
+# Existing: STANDING_CONFIG, LEG_JOINTS, etc.
 
 # Straight-leg joint targets used for initialization/standing checks.
 # Hip roll widened to help target a wide foot gap when combined with IK stance solve.
@@ -6,6 +7,7 @@
 STANDING_CONFIG: Dict[str, float] = {
     "leg_l1_joint": -0.22,  # push to wider stance (within limit -0.20..0.50)
     "leg_l2_joint": 0.0,
     "leg_l3_joint": 0.0,
     "leg_l4_joint": 0.0,
     "leg_l5_joint": 0.0,
     "leg_r1_joint": 0.22,
     "leg_r2_joint": 0.0,
     "leg_r3_joint": 0.0,
     "leg_r4_joint": 0.0,
     "leg_r5_joint": 0.0,
 }
+
+# Crouch stance candidates to reduce COM height and increase passive stability.
+# Note: knee sign can be model-dependent, so we provide both pos/neg variants for sweep.
+_CROUCH_KNEE = 0.6
+_CROUCH_ANKLE = 0.3
+
+CROUCH_CONFIG_POS: Dict[str, float] = {
+    "leg_l1_joint": -0.22,
+    "leg_l2_joint": 0.0,
+    "leg_l3_joint": +_CROUCH_KNEE,
+    "leg_l4_joint": +_CROUCH_KNEE,
+    "leg_l5_joint": -_CROUCH_ANKLE,
+    "leg_r1_joint": 0.22,
+    "leg_r2_joint": 0.0,
+    "leg_r3_joint": +_CROUCH_KNEE,
+    "leg_r4_joint": +_CROUCH_KNEE,
+    "leg_r5_joint": -_CROUCH_ANKLE,
+}
+
+CROUCH_CONFIG_NEG: Dict[str, float] = {
+    "leg_l1_joint": -0.22,
+    "leg_l2_joint": 0.0,
+    "leg_l3_joint": -_CROUCH_KNEE,
+    "leg_l4_joint": -_CROUCH_KNEE,
+    "leg_l5_joint": +_CROUCH_ANKLE,
+    "leg_r1_joint": 0.22,
+    "leg_r2_joint": 0.0,
+    "leg_r3_joint": -_CROUCH_KNEE,
+    "leg_r4_joint": -_CROUCH_KNEE,
+    "leg_r5_joint": +_CROUCH_ANKLE,
+}
+
+STANCE_CONFIGS: Dict[str, Dict[str, float]] = {
+    "standing": STANDING_CONFIG,
+    "crouch_pos": CROUCH_CONFIG_POS,
+    "crouch_neg": CROUCH_CONFIG_NEG,
+}
+
+def get_stance_config(name: str) -> Dict[str, float]:
+    return STANCE_CONFIGS.get(str(name), STANDING_CONFIG)
```

---

# 2) `config/agent_tuning.yaml`（stance指定を追加）

```diff
--- a/config/agent_tuning.yaml
+++ b/config/agent_tuning.yaml
@@ -1,6 +1,12 @@
 controller:
   type: two_stage
+  # Stance target used to build q_ref.
+  # Options: standing, crouch_pos, crouch_neg
+  stance: standing
+
   warmup_seconds: 1.0
   blend_seconds: 0.2
   position:
     kp: 30.0
     kd: 1.0
   torque:
     kp: 40.0
     kd: 1.5
     tau_limit: 60.0
     use_gravity_comp: false
     gravity_scale: 1.0
     kd_blend_factor: 2.0
```

---

# 3) `src/main_simulation.py`（standing-pd-ext で stance を読み、q_ref を差し替え）

> 既に standing-pd-ext は `sim.connect(enable_stable_contacts=True)` になっている前提で、**q_ref（target_positions）の作り方だけ差し替え**ます。

```diff
--- a/src/main_simulation.py
+++ b/src/main_simulation.py
@@ -1230,6 +1230,7 @@
-    from robot_constants import STANDING_CONFIG
+    from robot_constants import STANDING_CONFIG, get_stance_config
 
@@ -1265,9 +1266,15 @@
-    target_positions = dict(STANDING_CONFIG)
+    # stance selection (default: standing)
+    ctrl_cfg = (agent_cfg.get("controller") or {}) if isinstance(agent_cfg, dict) else {}
+    stance_name = ctrl_cfg.get("stance", "standing")
+    target_positions = dict(get_stance_config(stance_name))
 
     # reset / init
     sim.reset_robot(position=[0, 0, BASE_HEIGHT], joint_positions=target_positions)
 
+    # include stance in meta (ext_runner already writes meta)
+    if isinstance(run_meta, dict):
+        run_meta.setdefault("controller", {})
+        if isinstance(run_meta["controller"], dict):
+            run_meta["controller"]["stance"] = str(stance_name)
```

> ※ `agent_cfg`/`run_meta` の変数名があなたの現行と違う場合があるので、ここは **standing-pd-ext の YAML 読み込み直後**のブロックに合わせて当ててください（「controller.type を見て two_stage を選択」している箇所の近くが一番安全です）。

---

# 4) `scripts/sweep_tuning.py`（`--stance` を追加して探索対象に）

```diff
--- a/scripts/sweep_tuning.py
+++ b/scripts/sweep_tuning.py
@@ -145,6 +145,7 @@
     ap.add_argument("--pos-kp", default=None, help="Comma-separated warmup position kp candidates (two_stage)")
     ap.add_argument("--pos-kd", default=None, help="Comma-separated warmup position kd candidates (two_stage)")
+    ap.add_argument("--stance", default="standing", help="Comma-separated stance candidates: standing,crouch_pos,crouch_neg")
     ap.add_argument("--grid-sample", default="spread", choices=["spread", "random"], help="How to downsample grid when --trials < full grid size")
 
@@ -190,6 +191,7 @@
     pos_kps = _mk_grid(parse_floats(args.pos_kp)) if args.pos_kp is not None else None
     pos_kds = _mk_grid(parse_floats(args.pos_kd)) if args.pos_kd is not None else None
+    stances = [s.strip() for s in str(args.stance).split(",") if s.strip()]
 
@@ -205,7 +207,7 @@
         _pos_kps = pos_kps if pos_kps is not None else [None]
         _pos_kds = pos_kds if pos_kds is not None else [None]
         all_params = list(
             itertools.product(
-                kps, kds, taus, dts, settles, warmups, blends, grav_flags, grav_scales, kd_blends, _pos_kps, _pos_kds
+                kps, kds, taus, dts, settles, warmups, blends, grav_flags, grav_scales, kd_blends, _pos_kps, _pos_kds, stances
             )
         )
@@ -213,7 +215,7 @@
         if len(all_params) > max_grid:
             all_params = _downsample_grid(all_params, max_grid)
-        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend, pos_kp, pos_kd in all_params:
+        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend, pos_kp, pos_kd, stance in all_params:
             planned_params.append(
                 {
@@ -225,6 +227,7 @@
                     "kd_blend_factor": float(kd_blend),
                     "pos_kp": float(pos_kp) if pos_kp is not None else None,
                     "pos_kd": float(pos_kd) if pos_kd is not None else None,
+                    "stance": str(stance),
                 }
             )
@@ -244,6 +247,7 @@
                     "kd_blend_factor": float(random.choice(kd_blends)),
                     "pos_kp": float(random.choice(pos_kps)) if pos_kps is not None else None,
                     "pos_kd": float(random.choice(pos_kds)) if pos_kds is not None else None,
+                    "stance": str(random.choice(stances)),
                 }
             )
 
@@ -270,6 +274,8 @@
         if ctrl_type == "two_stage":
+            if p.get("stance", None) is not None:
+                cfg_ctrl["stance"] = str(p["stance"])
             cfg_ctrl["warmup_seconds"] = float(p.get("warmup_seconds", cfg_ctrl.get("warmup_seconds", 1.0)))
             cfg_ctrl["blend_seconds"] = float(p.get("blend_seconds", cfg_ctrl.get("blend_seconds", 0.0)))
             pos_block = dict(cfg_ctrl.get("position") or {})
@@ -456,6 +462,8 @@
     if str(best_cfg_ctrl.get("type", "torque_pd")) == "two_stage":
+        if best.params.get("stance", None) is not None:
+            best_cfg_ctrl["stance"] = str(best.params["stance"])
         best_cfg_ctrl["warmup_seconds"] = float(best.params.get("warmup_seconds", best_cfg_ctrl.get("warmup_seconds", 1.0)))
         best_cfg_ctrl["blend_seconds"] = float(best.params.get("blend_seconds", best_cfg_ctrl.get("blend_seconds", 0.0)))
```

---

# 5) `scripts/compare_runs.py`（stance列表示＋フィルタ）

```diff
--- a/scripts/compare_runs.py
+++ b/scripts/compare_runs.py
@@ -18,6 +18,7 @@
 class RunSummary:
@@
     pos_kp: Optional[float]
     pos_kd: Optional[float]
+    stance: Optional[str]
     tau_clip_frac_mean: Optional[float]
@@
 def _summarize_run(path: str) -> RunSummary:
@@
     pos_kp = _safe_get(meta, ["controller", "position", "kp"], default=None)
     pos_kd = _safe_get(meta, ["controller", "position", "kd"], default=None)
+    stance = _safe_get(meta, ["controller", "stance"], default=None)
@@
     return RunSummary(
@@
         pos_kp=pos_kp,
         pos_kd=pos_kd,
+        stance=str(stance) if stance is not None else None,
@@
 def _print_table(runs, limit):
     headers = [
@@
+        "stance",
         "pos_kp",
         "pos_kd",
@@
     rows.append([
@@
+        (r.stance or "-"),
         _fmt(r.pos_kp, 2),
         _fmt(r.pos_kd, 2),
@@
 def main():
@@
+    ap.add_argument("--stance", default="any", choices=["any", "standing", "crouch_pos", "crouch_neg"])
@@
     args = ap.parse_args()
@@
+    if args.stance != "any":
+        runs = [r for r in runs if (r.stance or "") == args.stance]
```

---

## これを入れた後に回す “最短” コマンド

まずは position-only 切り分けを stance だけで当てにいきます（符号問題も解決するため）。

```bash
cd /workspace/hunter

python3 scripts/sweep_tuning.py \
  --mode grid --grid-sample spread \
  --trials 12 --repeats 1 --no-gui \
  --control-dt 0.01 --settle 300 \
  --warmup 999 --blend 0.0 --grav 0 \
  --stance standing,crouch_pos,crouch_neg \
  --pos-kp 120,240 \
  --pos-kd 1.0,2.0
```

確認：

```bash
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --limit 20
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by qerr --limit 20
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --stance crouch_pos --sort-by score --limit 20
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --stance crouch_neg --sort-by score --limit 20
python3 scripts/analyze_abort_reasons.py --summary runs/sweep_summary.json --limit 12 --min-trials 3
```

---

## 期待する変化

* crouch のどちらかで **survival が 1s 超え始める**（まずこれがゴール）
* q_err_rms は引き続き小さいまま＝追従OK
* tilt の出方（pitch/roll）と base_z_min が改善方向に動く

---

もし crouch を入れても 1秒壁が破れない場合は、次は **BASE_HEIGHT を下げる sweep**（高さが高すぎて初期から不安定）を同じ仕組みで入れるのが次手です。

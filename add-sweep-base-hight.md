ここまでのログは決定打です。
**ct_any≈0.02（接触が取れる制御更新が2%）**は、ほぼ「ロボットが地面に乗っていない／跳ねている／初期から浮き気味で落下→跳ね→空中時間が長い」系の挙動です。
この状態だと、どれだけ q_ref に追従しても **接触が無いので姿勢は制御できず、ロールが発散**します。

次のステップは **BASE_HEIGHT を sweep して「安定して接触が成立する高さ」を当てる**のが最優先です。
（足幅や摩擦の前に、まず“地面に乗る”条件を作る）

---

# 次ステップパッチ：BASE_HEIGHT を YAML/Sweep で駆動できるようにする

## 方針

* いま `BASE_HEIGHT` を定数で使っている箇所（load_robot / reset_robot）を、

  * `config/agent_tuning.yaml: runner.base_height`（デフォルトは従来値）
  * `scripts/sweep_tuning.py --base-height ...` で trial ごとに上書き
    で差し替えます。
* meta に base_height を入れて compare で見えるようにします。

---

## 1) `config/agent_tuning.yaml` に runner.base_height を追加

```diff
--- a/config/agent_tuning.yaml
+++ b/config/agent_tuning.yaml
@@ -1,3 +1,8 @@
+runner:
+  # Override initial base height (z). Default keeps legacy behavior.
+  # This is critical when contact is mostly absent (ct_any ~ 0).
+  base_height: null
+
 controller:
   type: two_stage
   stance: standing
```

`null` の場合は従来 `BASE_HEIGHT` を使う、という扱いにします。

---

## 2) `scripts/sweep_tuning.py` に `--base-height` を追加して trial に反映

```diff
--- a/scripts/sweep_tuning.py
+++ b/scripts/sweep_tuning.py
@@ -152,6 +152,7 @@
     ap.add_argument("--max-roll", default=None, help="Comma-separated safety max_roll (rad) overrides")
     ap.add_argument("--max-pitch", default=None, help="Comma-separated safety max_pitch (rad) overrides")
+    ap.add_argument("--base-height", default=None, help="Comma-separated runner base_height overrides (m)")
     ap.add_argument("--grid-sample", default="spread", choices=["spread", "random"], help="How to downsample grid when --trials < full grid size")
@@ -203,6 +204,7 @@
     max_rolls = _mk_grid(parse_floats(args.max_roll)) if args.max_roll is not None else None
     max_pitches = _mk_grid(parse_floats(args.max_pitch)) if args.max_pitch is not None else None
+    base_heights = _mk_grid(parse_floats(args.base_height)) if args.base_height is not None else None
@@ -218,6 +220,7 @@
         _mr = max_rolls if max_rolls is not None else [None]
         _mp = max_pitches if max_pitches is not None else [None]
+        _bh = base_heights if base_heights is not None else [None]
         all_params = list(
             itertools.product(
                 kps, kds, taus, dts, settles, warmups, blends, grav_flags, grav_scales, kd_blends,
-                _pos_kps, _pos_kds, stances, _ck, _ca, _mr, _mp
+                _pos_kps, _pos_kds, stances, _ck, _ca, _mr, _mp, _bh
             )
         )
@@ -227,7 +230,7 @@
         if len(all_params) > max_grid:
             all_params = _downsample_grid(all_params, max_grid)
-        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend, pos_kp, pos_kd, stance, ck, ca, mr, mp in all_params:
+        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend, pos_kp, pos_kd, stance, ck, ca, mr, mp, bh in all_params:
             planned_params.append(
                 {
@@ -246,6 +249,7 @@
                     "max_roll": float(mr) if mr is not None else None,
                     "max_pitch": float(mp) if mp is not None else None,
+                    "base_height": float(bh) if bh is not None else None,
                 }
             )
@@ -268,6 +272,7 @@
                     "max_roll": float(random.choice(max_rolls)) if max_rolls is not None else None,
                     "max_pitch": float(random.choice(max_pitches)) if max_pitches is not None else None,
+                    "base_height": float(random.choice(base_heights)) if base_heights is not None else None,
                 }
             )
@@ -320,6 +325,16 @@
         # safety overrides (optional)
         cfg_safety = dict(cfg.get("safety") or {})
@@
         cfg["safety"] = cfg_safety
+
+        # runner overrides (optional)
+        cfg_runner = dict(cfg.get("runner") or {})
+        if p.get("base_height", None) is not None:
+            cfg_runner["base_height"] = float(p["base_height"])
+        cfg["runner"] = cfg_runner
```

---

## 3) `src/main_simulation.py`（standing-pd-ext で base_height を読む）

standing-pd-ext の初期化で、`load_robot(start_position=[0,0,BASE_HEIGHT])` と `reset_robot(position=[0,0,BASE_HEIGHT], ...)` を置き換えます。

```diff
--- a/src/main_simulation.py
+++ b/src/main_simulation.py
@@ -1235,6 +1235,18 @@
     # agent_cfg loaded from config/agent_tuning.yaml (already in your code)
     ctrl_cfg = (agent_cfg.get("controller") or {}) if isinstance(agent_cfg, dict) else {}
+    runner_cfg = (agent_cfg.get("runner") or {}) if isinstance(agent_cfg, dict) else {}
+
+    # base height override (if provided), else fallback to legacy BASE_HEIGHT constant
+    base_h = runner_cfg.get("base_height", None)
+    try:
+        base_h = float(base_h) if base_h is not None else None
+    except Exception:
+        base_h = None
+    if base_h is None:
+        base_h = BASE_HEIGHT
 
@@ -1248,7 +1260,7 @@
-    sim.load_robot(start_position=[0, 0, BASE_HEIGHT])
+    sim.load_robot(start_position=[0, 0, base_h])
@@ -1275,7 +1287,7 @@
-    sim.reset_robot(position=[0, 0, BASE_HEIGHT], joint_positions=target_positions)
+    sim.reset_robot(position=[0, 0, base_h], joint_positions=target_positions)
 
     if isinstance(run_meta, dict):
         run_meta.setdefault("runner", {})
         if isinstance(run_meta["runner"], dict):
+            run_meta["runner"]["base_height"] = float(base_h)
             # (existing meta fields…)
             pass
```

---

## 4) `scripts/compare_runs.py` に base_height 列を追加（見やすく）

```diff
--- a/scripts/compare_runs.py
+++ b/scripts/compare_runs.py
@@ -18,6 +18,7 @@ class RunSummary:
     stance: Optional[str]
     crouch_knee: Optional[float]
     crouch_ankle: Optional[float]
+    base_height: Optional[float]
@@
 def _summarize_run(path: str) -> RunSummary:
@@
+    base_h = _safe_get(meta, ["runner", "base_height"], default=None)
@@
+    try:
+        base_h = float(base_h) if base_h is not None else None
+    except Exception:
+        base_h = None
@@
     return RunSummary(
@@
         crouch_ankle=crouch_ankle,
+        base_height=base_h,
@@
 def _print_table(...):
     headers = [
@@
+        "base_h",
         "stance",
         "cknee",
         "cankle",
@@
     rows.append([
+        _fmt(r.base_height, 3),
         (r.stance or "-"),
         _fmt(r.crouch_knee, 3),
         _fmt(r.crouch_ankle, 3),
```

---

# 次に回すコマンド（BASE_HEIGHT sweep：最小）

ct_any が 0.02 なので、まず “接触が増える高さ” を探します。
範囲は仮で、まず **現状±0.1m**くらいで粗く。

```bash
cd /workspace/hunter

python3 scripts/sweep_tuning.py \
  --mode grid --grid-sample spread \
  --trials 12 --repeats 1 --no-gui --duration 3 \
  --control-dt 0.01 --settle 300 \
  --warmup 999 --blend 0.0 --grav 0 \
  --stance crouch_pos --crouch-knee 0.70 --crouch-ankle 0.30 \
  --pos-kp 120 --pos-kd 1.0 \
  --base-height 0.45,0.50,0.55,0.60,0.65,0.70
```

見る：

```bash
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by mtime --limit 20
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --limit 20
```

**注目する列**

* `ct_any`（接触が取れる率）が上がるか
* `foot_hit` が上がるか
* `top_contact` が何になるか
* survival が 1秒を越えるか

---

## 期待する “当たり”

* `ct_any` が 0.02 → 0.5 以上に増える base_height が必ずどこかに出ます
  （出なければ、地面/衝突フィルタ/接触取得範囲が根本的におかしい可能性が出ます）

---

これが決まると、次は

* ct_any を最大化する高さに固定して stance/足幅/摩擦へ
  に進めます。

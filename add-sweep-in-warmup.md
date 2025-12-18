了解。今回の結果はかなり重要で、

* **tilt 100%（DONE 0）**
* **clip_mean が極小（= 飽和してない）**

なので、今は「トルクが足りない/飽和で倒れる」ではなく、ほぼ **“姿勢が崩れ始めた瞬間に支えきれてない（＝初期・切替・剛性不足）”**側です。

その中でも一番ありがちな原因はこれです：

> **warmup の position 制御が弱すぎて、立位姿勢が作れる前に倒れる**
> （あなたのデフォルト `position.kp=0.3, kd=0.1` は、物理的にはほぼ“柔らかい糸”です）

なので次ステップは **warmup(position) のゲインを sweep 対象に追加**して、「まず立つ剛性」を作りに行きます。

---

# 次ステップのパッチ一式（unified diff）

対象：

* `config/agent_tuning.yaml`（warmup position ゲインのデフォルト強化）
* `scripts/sweep_tuning.py`（`--pos-kp`, `--pos-kd` を追加して探索対象化）
* `scripts/compare_runs.py`（表に `pos_kp`, `pos_kd` 列を追加＋フィルタ）

※ `main_simulation.py` はすでに `controller.position.kp/kd` を TwoStage に渡している前提なので **変更なし**です（あなたの実装説明どおり）。

---

## 1) `config/agent_tuning.yaml`（positionゲインを現実値へ）

```diff
--- a/config/agent_tuning.yaml
+++ b/config/agent_tuning.yaml
@@ -10,10 +10,14 @@
   # Warmup stage (POSITION_CONTROL)
   position:
-    kp: 0.3
-    kd: 0.1
+    # NOTE: 0.3/0.1 is usually far too soft; robot collapses before hold.
+    # Start with "stiff enough to stand", then tune down if needed.
+    kp: 30.0
+    kd: 1.0
```

---

## 2) `scripts/sweep_tuning.py`（posゲイン探索を追加）

```diff
--- a/scripts/sweep_tuning.py
+++ b/scripts/sweep_tuning.py
@@ -140,6 +140,8 @@
     ap.add_argument("--warmup", default="1.0", help="Comma-separated warmup_seconds candidates (two_stage)")
     ap.add_argument("--blend", default="0.0", help="Comma-separated blend_seconds candidates (two_stage)")
     ap.add_argument("--grav", default="0,1", help="Comma-separated use_gravity_comp candidates: 0 or 1 (two_stage)")
     ap.add_argument("--grav-scale", default="1.0", help="Comma-separated gravity_scale candidates (two_stage)")
     ap.add_argument("--kd-blend", default="1.0", help="Comma-separated kd_blend_factor candidates (two_stage)")
+    ap.add_argument("--pos-kp", default=None, help="Comma-separated warmup position kp candidates (two_stage)")
+    ap.add_argument("--pos-kd", default=None, help="Comma-separated warmup position kd candidates (two_stage)")
     ap.add_argument("--grid-sample", default="spread", choices=["spread", "random"], help="How to downsample grid when --trials < full grid size")
 
@@ -176,6 +178,15 @@
     warmups = _mk_grid(parse_floats(args.warmup))
     blends = _mk_grid(parse_floats(args.blend))
     grav_flags = [int(float(x)) for x in args.grav.split(",") if x.strip()]
     grav_scales = _mk_grid(parse_floats(args.grav_scale))
     kd_blends = _mk_grid(parse_floats(args.kd_blend))
+
+    # Position gains: if not provided, keep as single "current YAML" value (handled later).
+    pos_kps = _mk_grid(parse_floats(args.pos_kp)) if args.pos_kp is not None else None
+    pos_kds = _mk_grid(parse_floats(args.pos_kd)) if args.pos_kd is not None else None
 
@@ -190,7 +201,16 @@
     planned_params: List[Dict[str, Any]] = []
 
     if args.mode == "grid":
-        all_params = list(itertools.product(kps, kds, taus, dts, settles, warmups, blends, grav_flags, grav_scales, kd_blends))
+        # If pos gains are unspecified, treat as 1 choice (we'll read from YAML later).
+        _pos_kps = pos_kps if pos_kps is not None else [None]
+        _pos_kds = pos_kds if pos_kds is not None else [None]
+        all_params = list(
+            itertools.product(
+                kps, kds, taus, dts, settles, warmups, blends, grav_flags, grav_scales, kd_blends, _pos_kps, _pos_kds
+            )
+        )
         max_grid = max(1, args.trials)
         if len(all_params) > max_grid:
             all_params = _downsample_grid(all_params, max_grid)
-        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend in all_params:
+        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend, pos_kp, pos_kd in all_params:
             planned_params.append(
                 {
                     "kp": kp,
                     "kd": kd,
                     "tau_limit": tau,
                     "control_dt": dt,
                     "settle_steps": settle,
                     "warmup_seconds": warmup,
                     "blend_seconds": blend,
                     "use_gravity_comp": bool(int(grav)),
                     "gravity_scale": float(gscale),
                     "kd_blend_factor": float(kd_blend),
+                    "pos_kp": float(pos_kp) if pos_kp is not None else None,
+                    "pos_kd": float(pos_kd) if pos_kd is not None else None,
                 }
             )
@@ -212,6 +232,10 @@
         for _ in range(args.trials):
             planned_params.append(
                 {
                     "kp": random.choice(kps),
                     "kd": random.choice(kds),
                     "tau_limit": random.choice(taus),
                     "control_dt": random.choice(dts),
                     "settle_steps": random.choice(settles),
                     "warmup_seconds": random.choice(warmups),
                     "blend_seconds": random.choice(blends),
                     "use_gravity_comp": bool(int(random.choice(grav_flags))),
                     "gravity_scale": float(random.choice(grav_scales)),
                     "kd_blend_factor": float(random.choice(kd_blends)),
+                    "pos_kp": float(random.choice(pos_kps)) if pos_kps is not None else None,
+                    "pos_kd": float(random.choice(pos_kds)) if pos_kds is not None else None,
                 }
             )
 
@@ -240,6 +264,18 @@
         ctrl_type = str(cfg_ctrl.get("type", "torque_pd"))
         if ctrl_type == "two_stage":
             cfg_ctrl["warmup_seconds"] = float(p.get("warmup_seconds", cfg_ctrl.get("warmup_seconds", 1.0)))
             cfg_ctrl["blend_seconds"] = float(p.get("blend_seconds", cfg_ctrl.get("blend_seconds", 0.0)))
+
+            # Apply position gains only if trial provided them (otherwise keep YAML as-is).
+            pos_block = dict(cfg_ctrl.get("position") or {})
+            if p.get("pos_kp", None) is not None:
+                pos_block["kp"] = float(p["pos_kp"])
+            if p.get("pos_kd", None) is not None:
+                pos_block["kd"] = float(p["pos_kd"])
+            cfg_ctrl["position"] = pos_block
+
             torque_block = dict(cfg_ctrl.get("torque") or {})
             torque_block["kp"] = float(p["kp"])
             torque_block["kd"] = float(p["kd"])
             torque_block["tau_limit"] = float(p["tau_limit"])
             torque_block["use_gravity_comp"] = bool(p.get("use_gravity_comp", torque_block.get("use_gravity_comp", False)))
             torque_block["gravity_scale"] = float(p.get("gravity_scale", torque_block.get("gravity_scale", 1.0)))
             torque_block["kd_blend_factor"] = float(p.get("kd_blend_factor", torque_block.get("kd_blend_factor", 1.0)))
             cfg_ctrl["torque"] = torque_block
@@ -410,6 +446,18 @@
     if str(best_cfg_ctrl.get("type", "torque_pd")) == "two_stage":
         best_cfg_ctrl["warmup_seconds"] = float(best.params.get("warmup_seconds", best_cfg_ctrl.get("warmup_seconds", 1.0)))
         best_cfg_ctrl["blend_seconds"] = float(best.params.get("blend_seconds", best_cfg_ctrl.get("blend_seconds", 0.0)))
+
+        pos_block = dict(best_cfg_ctrl.get("position") or {})
+        if best.params.get("pos_kp", None) is not None:
+            pos_block["kp"] = float(best.params["pos_kp"])
+        if best.params.get("pos_kd", None) is not None:
+            pos_block["kd"] = float(best.params["pos_kd"])
+        best_cfg_ctrl["position"] = pos_block
+
         torque_block = dict(best_cfg_ctrl.get("torque") or {})
         torque_block["kp"] = float(best.params["kp"])
         torque_block["kd"] = float(best.params["kd"])
         torque_block["tau_limit"] = float(best.params["tau_limit"])
         torque_block["use_gravity_comp"] = bool(best.params.get("use_gravity_comp", torque_block.get("use_gravity_comp", False)))
         torque_block["gravity_scale"] = float(best.params.get("gravity_scale", torque_block.get("gravity_scale", 1.0)))
         torque_block["kd_blend_factor"] = float(best.params.get("kd_blend_factor", torque_block.get("kd_blend_factor", 1.0)))
         best_cfg_ctrl["torque"] = torque_block
```

---

## 3) `scripts/compare_runs.py`（posゲイン列＋フィルタ）

`meta.controller.position.kp/kd` を表示し、必要なら絞れるようにします。

```diff
--- a/scripts/compare_runs.py
+++ b/scripts/compare_runs.py
@@ -18,6 +18,8 @@
 class RunSummary:
     path: str
     mtime: float
     status: str
     score: Optional[float]
     grav_on: Optional[bool]
     grav_scale: Optional[float]
     blend_seconds: Optional[float]
     kd_blend_factor: Optional[float]
+    pos_kp: Optional[float]
+    pos_kd: Optional[float]
     tau_clip_frac_mean: Optional[float]
     tau_clip_frac_max: Optional[float]
     survival_time: Optional[float]
@@ -96,6 +98,9 @@
     grav_on = _safe_get(meta, ["controller", "torque", "use_gravity_comp"], default=None)
     grav_scale = _safe_get(meta, ["controller", "torque", "gravity_scale"], default=None)
     blend_seconds = _safe_get(meta, ["controller", "blend_seconds"], default=None)
     kd_blend_factor = _safe_get(meta, ["controller", "torque", "kd_blend_factor"], default=None)
+    pos_kp = _safe_get(meta, ["controller", "position", "kp"], default=None)
+    pos_kd = _safe_get(meta, ["controller", "position", "kd"], default=None)
 
@@ -124,6 +129,14 @@
     try:
         kd_blend_factor = float(kd_blend_factor) if kd_blend_factor is not None else None
     except Exception:
         kd_blend_factor = None
+    try:
+        pos_kp = float(pos_kp) if pos_kp is not None else None
+    except Exception:
+        pos_kp = None
+    try:
+        pos_kd = float(pos_kd) if pos_kd is not None else None
+    except Exception:
+        pos_kd = None
 
     return RunSummary(
@@ -135,6 +148,8 @@
         grav_on=grav_on,
         grav_scale=grav_scale,
         blend_seconds=blend_seconds,
         kd_blend_factor=kd_blend_factor,
+        pos_kp=pos_kp,
+        pos_kd=pos_kd,
         tau_clip_frac_mean=clip_mean,
         tau_clip_frac_max=clip_max,
         survival_time=float(st) if st is not None else None,
@@ -160,6 +175,8 @@
         "g_scale",
         "blend",
         "kd_blend",
+        "pos_kp",
+        "pos_kd",
         "clip_mean",
         "clip_max",
         "survival_s",
@@ -181,6 +198,8 @@
             _fmt_grav_on(r.grav_on),
             _fmt(r.grav_scale, 3),
             _fmt(r.blend_seconds, 3),
             _fmt(r.kd_blend_factor, 3),
+            _fmt(r.pos_kp, 2),
+            _fmt(r.pos_kd, 2),
             _fmt(r.tau_clip_frac_mean, 3),
             _fmt(r.tau_clip_frac_max, 3),
             _fmt(r.survival_time, 3),
@@ -210,6 +229,10 @@
     ap.add_argument("--gscale-min", type=float, default=None)
     ap.add_argument("--gscale-max", type=float, default=None)
     ap.add_argument("--blend-min", type=float, default=None)
     ap.add_argument("--blend-max", type=float, default=None)
     ap.add_argument("--kdblend-min", type=float, default=None)
     ap.add_argument("--kdblend-max", type=float, default=None)
+    ap.add_argument("--poskp-min", type=float, default=None, help="Filter: position kp >= this")
+    ap.add_argument("--poskp-max", type=float, default=None, help="Filter: position kp <= this")
+    ap.add_argument("--poskd-min", type=float, default=None, help="Filter: position kd >= this")
+    ap.add_argument("--poskd-max", type=float, default=None, help="Filter: position kd <= this")
     args = ap.parse_args()
@@ -252,6 +275,18 @@
     if args.kdblend_min is not None:
         runs = [r for r in runs if r.kd_blend_factor is not None and r.kd_blend_factor >= args.kdblend_min]
     if args.kdblend_max is not None:
         runs = [r for r in runs if r.kd_blend_factor is not None and r.kd_blend_factor <= args.kdblend_max]
+
+    if args.poskp_min is not None:
+        runs = [r for r in runs if r.pos_kp is not None and r.pos_kp >= args.poskp_min]
+    if args.poskp_max is not None:
+        runs = [r for r in runs if r.pos_kp is not None and r.pos_kp <= args.poskp_max]
+    if args.poskd_min is not None:
+        runs = [r for r in runs if r.pos_kd is not None and r.pos_kd >= args.poskd_min]
+    if args.poskd_max is not None:
+        runs = [r for r in runs if r.pos_kd is not None and r.pos_kd <= args.poskd_max]
```

---

# 次に回すコマンド（「warmup剛性」を作る最小 sweep）

まずは **pos_kp/pos_kd を動かして tilt 100% を崩す**のが目的です。

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
  --kp 20 \
  --kd 4.5 \
  --kd-blend 3.0 \
  --tau 150 \
  --pos-kp 10,30,60,120 \
  --pos-kd 0.5,1.0,2.0,4.0
```

結果を見る：

```bash
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --grav on --sort-by score --limit 20
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --grav on --sort-by clip --limit 20
python3 scripts/analyze_abort_reasons.py --summary runs/sweep_summary.json --limit 12 --min-trials 3
```

---

## ここでの期待

* survival が 1s → 2〜5s に伸び始める
* tilt 100% でも “落ちるまでが伸びる” なら正しい方向
* DONE が出たら、その設定を seed 固定で 10s 回して再現性確認へ



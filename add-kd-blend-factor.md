以下に、**kd_blend_factor（blend中だけ torque側の kd を増やす）**を導入するための **パッチ一式（unified diff）**をまとめて出します。

対象：

* `src/ext_controller_two_stage.py`（実装）
* `src/main_simulation.py`（YAML→TwoStage へ受け渡し）
* `config/agent_tuning.yaml`（設定追加）
* `scripts/sweep_tuning.py`（`--kd-blend` 追加＆探索対象化）
* `scripts/compare_runs.py`（`blend` と `kd_blend` 列表示＋フィルタ）

---

## 1) `src/ext_controller_two_stage.py`

```diff
--- a/src/ext_controller_two_stage.py
+++ b/src/ext_controller_two_stage.py
@@ -16,6 +16,7 @@
 class TorqueStageGains:
     kp: float = 40.0
     kd: float = 1.5
     tau_limit: float = 60.0
     use_gravity_comp: bool = False
     gravity_scale: float = 1.0
+    kd_blend_factor: float = 1.0
 
 
 class TwoStagePostureController:
@@ -30,6 +31,7 @@
         q_ref: np.ndarray,
         *,
         robot_id: int,
         warmup_seconds: float = 1.0,
         blend_seconds: float = 0.0,
         position_gains: PositionStageGains = PositionStageGains(),
         torque_gains: TorqueStageGains = TorqueStageGains(),
@@ -108,13 +110,23 @@
         if self._in_blend(t):
             a = self._blend_alpha(t)
-            # Torque PD (the same as hold), but ramped in gradually
-            tau_pd = self.tau_g.kp * (self.q_ref - obs.q) - self.tau_g.kd * obs.dq
+            # Torque PD (ramped in gradually). Increase damping only during blend.
+            kd_eff = float(self.tau_g.kd) * float(self.tau_g.kd_blend_factor)
+            tau_pd = self.tau_g.kp * (self.q_ref - obs.q) - kd_eff * obs.dq
             tau_ff = (tau_pd + self._gravity_ff(obs)) * a
             tau_ff = np.clip(tau_ff, -self.tau_g.tau_limit, self.tau_g.tau_limit)
 
             cmds = {}
             for i, j in enumerate(LEG_JOINTS):
                 cmds[j] = {
                     "mode": "hybrid",
                     "position": float(self.q_ref[i]),
                     "velocity": 0.0,
                     "kp": float(self.pos_g.kp),
                     "kd": float(self.pos_g.kd),
                     "torque": float(tau_ff[i]),
                 }
             return cmds
```

---

## 2) `src/main_simulation.py`（two_stage 作成時に YAML の `kd_blend_factor` を反映）

```diff
--- a/src/main_simulation.py
+++ b/src/main_simulation.py
@@ -1270,6 +1270,7 @@
         if ctrl_type == "two_stage":
             from ext_controller_two_stage import (
                 TwoStagePostureController,
                 PositionStageGains,
                 TorqueStageGains,
             )
 
             warmup_seconds = float(ctrl_cfg.get("warmup_seconds", 1.0))
             blend_seconds = float(ctrl_cfg.get("blend_seconds", 0.0))
             pos_cfg = (ctrl_cfg.get("position") or {})
             tau_cfg = (ctrl_cfg.get("torque") or {})
             controller = TwoStagePostureController(
                 q_ref,
                 robot_id=sim.robot_id,
                 warmup_seconds=warmup_seconds,
                 blend_seconds=blend_seconds,
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
+                    kd_blend_factor=float(tau_cfg.get("kd_blend_factor", 1.0)),
                 ),
             )
```

---

## 3) `config/agent_tuning.yaml`（デフォルト追加）

```diff
--- a/config/agent_tuning.yaml
+++ b/config/agent_tuning.yaml
@@ -18,10 +18,13 @@
   torque:
     kp: 40.0
     kd: 1.5
     tau_limit: 60.0
     use_gravity_comp: false
     gravity_scale: 1.0
+    # Increase torque damping only during blend (warmup->hold transition).
+    # 1.0 means "no change". Typical sweep: 1.0, 1.5, 2.0, 3.0
+    kd_blend_factor: 2.0
```

---

## 4) `scripts/sweep_tuning.py`（`--kd-blend` 追加＆探索対象化）

```diff
--- a/scripts/sweep_tuning.py
+++ b/scripts/sweep_tuning.py
@@ -140,6 +140,7 @@
     ap.add_argument("--warmup", default="1.0", help="Comma-separated warmup_seconds candidates (two_stage)")
     ap.add_argument("--blend", default="0.0", help="Comma-separated blend_seconds candidates (two_stage)")
     ap.add_argument("--grav", default="0,1", help="Comma-separated use_gravity_comp candidates: 0 or 1 (two_stage)")
     ap.add_argument("--grav-scale", default="1.0", help="Comma-separated gravity_scale candidates (two_stage)")
+    ap.add_argument("--kd-blend", default="1.0", help="Comma-separated kd_blend_factor candidates (two_stage)")
 
@@ -176,6 +177,7 @@
     warmups = _mk_grid(parse_floats(args.warmup))
     blends = _mk_grid(parse_floats(args.blend))
     grav_flags = [int(float(x)) for x in args.grav.split(",") if x.strip()]
     grav_scales = _mk_grid(parse_floats(args.grav_scale))
+    kd_blends = _mk_grid(parse_floats(args.kd_blend))
 
@@ -186,7 +188,7 @@
     planned_params: List[Dict[str, Any]] = []
 
     if args.mode == "grid":
-        all_params = list(itertools.product(kps, kds, taus, dts, settles, warmups, blends, grav_flags, grav_scales))
+        all_params = list(itertools.product(kps, kds, taus, dts, settles, warmups, blends, grav_flags, grav_scales, kd_blends))
         max_grid = max(1, args.trials)
         if len(all_params) > max_grid:
             step = max(1, len(all_params) // max_grid)
             all_params = all_params[::step][:max_grid]
-        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale in all_params:
+        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend in all_params:
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
+                    "kd_blend_factor": float(kd_blend),
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
                     "blend_seconds": random.choice(blends),
                     "use_gravity_comp": bool(int(random.choice(grav_flags))),
                     "gravity_scale": float(random.choice(grav_scales)),
+                    "kd_blend_factor": float(random.choice(kd_blends)),
                 }
             )
 
@@ -240,6 +242,7 @@
         if ctrl_type == "two_stage":
             cfg_ctrl["warmup_seconds"] = float(p.get("warmup_seconds", cfg_ctrl.get("warmup_seconds", 1.0)))
             cfg_ctrl["blend_seconds"] = float(p.get("blend_seconds", cfg_ctrl.get("blend_seconds", 0.0)))
             torque_block = dict(cfg_ctrl.get("torque") or {})
             torque_block["kp"] = float(p["kp"])
             torque_block["kd"] = float(p["kd"])
             torque_block["tau_limit"] = float(p["tau_limit"])
             torque_block["use_gravity_comp"] = bool(p.get("use_gravity_comp", torque_block.get("use_gravity_comp", False)))
             torque_block["gravity_scale"] = float(p.get("gravity_scale", torque_block.get("gravity_scale", 1.0)))
+            torque_block["kd_blend_factor"] = float(p.get("kd_blend_factor", torque_block.get("kd_blend_factor", 1.0)))
             cfg_ctrl["torque"] = torque_block
         else:
             cfg_ctrl["kp"] = float(p["kp"])
             cfg_ctrl["kd"] = float(p["kd"])
             cfg_ctrl["tau_limit"] = float(p["tau_limit"])
 
@@ -410,6 +413,7 @@
     if str(best_cfg_ctrl.get("type", "torque_pd")) == "two_stage":
         best_cfg_ctrl["warmup_seconds"] = float(best.params.get("warmup_seconds", best_cfg_ctrl.get("warmup_seconds", 1.0)))
         best_cfg_ctrl["blend_seconds"] = float(best.params.get("blend_seconds", best_cfg_ctrl.get("blend_seconds", 0.0)))
         torque_block = dict(best_cfg_ctrl.get("torque") or {})
         torque_block["kp"] = float(best.params["kp"])
         torque_block["kd"] = float(best.params["kd"])
         torque_block["tau_limit"] = float(best.params["tau_limit"])
         torque_block["use_gravity_comp"] = bool(best.params.get("use_gravity_comp", torque_block.get("use_gravity_comp", False)))
         torque_block["gravity_scale"] = float(best.params.get("gravity_scale", torque_block.get("gravity_scale", 1.0)))
+        torque_block["kd_blend_factor"] = float(best.params.get("kd_blend_factor", torque_block.get("kd_blend_factor", 1.0)))
         best_cfg_ctrl["torque"] = torque_block
```

---

## 5) `scripts/compare_runs.py`（`blend` と `kd_blend` 列＋フィルタ）

> 既に `meta.controller.*` を読む実装が入っている前提で、列とフィルタを追加します。

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
+    blend_seconds: Optional[float]
+    kd_blend_factor: Optional[float]
     survival_time: Optional[float]
     tilt_max_abs: Optional[float]
     base_z_min: Optional[float]
     energy_abs_tau_dq: Optional[float]
@@ -66,6 +68,8 @@
     meta = payload.get("meta", {}) if isinstance(payload, dict) else {}
 
     st = _safe_get(metrics, ["survival_time"])
     tilt = _safe_get(metrics, ["tilt_max_abs"])
@@ -92,6 +96,10 @@
     grav_on = _safe_get(meta, ["controller", "torque", "use_gravity_comp"], default=None)
     grav_scale = _safe_get(meta, ["controller", "torque", "gravity_scale"], default=None)
+    blend_seconds = _safe_get(meta, ["controller", "blend_seconds"], default=None)
+    kd_blend_factor = _safe_get(meta, ["controller", "torque", "kd_blend_factor"], default=None)
+
     try:
         grav_on = bool(grav_on) if grav_on is not None else None
     except Exception:
         grav_on = None
@@ -102,6 +110,14 @@
         grav_scale = float(grav_scale) if grav_scale is not None else None
     except Exception:
         grav_scale = None
+    try:
+        blend_seconds = float(blend_seconds) if blend_seconds is not None else None
+    except Exception:
+        blend_seconds = None
+    try:
+        kd_blend_factor = float(kd_blend_factor) if kd_blend_factor is not None else None
+    except Exception:
+        kd_blend_factor = None
 
     return RunSummary(
         path=path,
         mtime=os.path.getmtime(path),
         status=str(result.get("status", "UNKNOWN")) if isinstance(result, dict) else "UNKNOWN",
         score=score,
         grav_on=grav_on,
         grav_scale=grav_scale,
+        blend_seconds=blend_seconds,
+        kd_blend_factor=kd_blend_factor,
         survival_time=float(st) if st is not None else None,
         tilt_max_abs=float(tilt) if tilt is not None else None,
         base_z_min=float(zmin) if zmin is not None else None,
         energy_abs_tau_dq=float(energy) if energy is not None else None,
@@ -126,6 +142,7 @@
     headers = [
         "rank",
         "time",
         "status",
         "score",
         "grav",
         "g_scale",
+        "blend",
+        "kd_blend",
         "survival_s",
         "tilt_max(rad)",
         "base_z_min",
         "energy",
@@ -141,6 +158,8 @@
             r.status,
             _fmt(r.score, 3),
             _fmt_grav_on(r.grav_on),
             _fmt(r.grav_scale, 3),
+            _fmt(r.blend_seconds, 3),
+            _fmt(r.kd_blend_factor, 3),
             _fmt(r.survival_time, 3),
             _fmt(r.tilt_max_abs, 3),
             _fmt(r.base_z_min, 3),
             _fmt(r.energy_abs_tau_dq, 3),
@@ -182,6 +201,10 @@
     ap.add_argument("--grav", default="any", choices=["any", "on", "off"])
     ap.add_argument("--gscale-min", type=float, default=None)
     ap.add_argument("--gscale-max", type=float, default=None)
+    ap.add_argument("--blend-min", type=float, default=None, help="Filter: blend_seconds >= this")
+    ap.add_argument("--blend-max", type=float, default=None, help="Filter: blend_seconds <= this")
+    ap.add_argument("--kdblend-min", type=float, default=None, help="Filter: kd_blend_factor >= this")
+    ap.add_argument("--kdblend-max", type=float, default=None, help="Filter: kd_blend_factor <= this")
     args = ap.parse_args()
@@ -214,6 +237,18 @@
     if args.gscale_min is not None:
         runs = [r for r in runs if r.grav_scale is not None and r.grav_scale >= args.gscale_min]
     if args.gscale_max is not None:
         runs = [r for r in runs if r.grav_scale is not None and r.grav_scale <= args.gscale_max]
+
+    if args.blend_min is not None:
+        runs = [r for r in runs if r.blend_seconds is not None and r.blend_seconds >= args.blend_min]
+    if args.blend_max is not None:
+        runs = [r for r in runs if r.blend_seconds is not None and r.blend_seconds <= args.blend_max]
+
+    if args.kdblend_min is not None:
+        runs = [r for r in runs if r.kd_blend_factor is not None and r.kd_blend_factor >= args.kdblend_min]
+    if args.kdblend_max is not None:
+        runs = [r for r in runs if r.kd_blend_factor is not None and r.kd_blend_factor <= args.kdblend_max]
```

---

# 実行例（Docker内）

まずは **切替の減衰**だけ見たいので、パラメータを固定しつつ `kd_blend_factor` を振るのが効率的です。

```bash
cd /workspace/hunter

# 最小グリッド：kd_blend_factor だけ変える（grav ON 推奨）
python3 scripts/sweep_tuning.py \
  --mode grid --trials 6 \
  --kp 40 --kd 1.5 --tau 60 \
  --control-dt 0.01 --settle 0 \
  --warmup 0.5 --blend 0.2 \
  --grav 1 --grav-scale 1.0 \
  --kd-blend 1.0,1.5,2.0,3.0 \
  --repeats 2 \
  --no-gui

# score順で “ON + blend + kd_blend” を確認
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --grav on --limit 15

# 例えば kd_blend>=2.0 だけに絞って比較
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --grav on --kdblend-min 2.0 --limit 10
```

この変更で、`tilt too large ...` の頻度が下がったり、survival が伸びたり、まれに DONE が出始めるのが典型です。

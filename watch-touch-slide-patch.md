ここまでの観測で「安全が厳しすぎて切られてるだけ」説は潰れましたね。結果がめちゃくちゃ綺麗で、

* max_roll を 0.70→0.85→1.00 と上げると **その値までロールが増大して** abort
* survival は 0.89→0.92→0.96 と少し伸びるが **1秒を超えない**
* **回復（収束）している兆候がない**

なので次のステップは **“ロール発散の原因” を突き止めて抑える**方向です。
この手の「静止姿勢なのにロールが増えていく」原因はほぼ次のどれかです：

1. **足が滑っている（横方向摩擦不足／接触剛性／接触点のズレ）**
2. **初期の足裏が水平でなく、横方向にトルクを生む（足首・足裏姿勢のミスマッチ）**
3. **ベース高さが合っておらず、足が押し込み/浮きで片脚支持になっている（左右接触の偏り）**

ここで重要なのは、もう「関節追従」はOKなので、見るべきは **接触（左右の接触力、foot位置、接触有無）**です。

---

# 次ステップ（最短で原因特定）：接触の左右差・滑りをログ＆集計する

やること：

* samples に以下を追加して metrics 化し、compare で見えるようにする

  * `fz_left`, `fz_right`（左右の垂直方向接触力）
  * `fxy_left`, `fxy_right`（水平成分の大きさ）
  * `f_ratio`（左右の荷重比：min/max）
  * `foot_dx`（左右足の横方向距離 or 足幅）
  * `slip_left/right`（足位置の時間差分：|Δx,Δy|）

これで

* 片側荷重が崩れていくなら **BASE_HEIGHT/姿勢**
* 水平力が大きく slip が増えるなら **摩擦/足裏/足首**
  が一発で分かります。

---

## パッチ一式（unified diff）

### 1) `src/ext_runner.py`（接触・足位置を samples に記録）

※あなたの obs 仕様どおり `raw_obs` には `contact_forces` と `foot_positions` が入っている前提です。

```diff
--- a/src/ext_runner.py
+++ b/src/ext_runner.py
@@ -1,6 +1,7 @@
 from ext_obs_adapter import adapt_obs
 from ext_safety import should_abort
 from ext_normalize import normalize_joint_commands
 from ext_metrics import compute_metrics
+import math
 
@@ -40,6 +41,42 @@ def _q_err_rms(obs, controller) -> Optional[float]:
     except Exception:
         return None
 
+def _norm2(xy) -> Optional[float]:
+    try:
+        x = float(xy[0]); y = float(xy[1])
+        return math.sqrt(x*x + y*y)
+    except Exception:
+        return None
+
+def _safe_vec3(v):
+    try:
+        return (float(v[0]), float(v[1]), float(v[2]))
+    except Exception:
+        return None
+
+def _extract_contact_and_feet(raw_obs: Dict[str, Any]) -> Dict[str, Any]:
+    """
+    raw_obs["contact_forces"]: {"left": np(3,), "right": np(3,)}  (sum force)
+    raw_obs["foot_positions"]: {"left": np(3,), "right": np(3,)}
+    """
+    out: Dict[str, Any] = {}
+    cf = raw_obs.get("contact_forces") or {}
+    fp = raw_obs.get("foot_positions") or {}
+    lf = _safe_vec3(cf.get("left")) if isinstance(cf, dict) else None
+    rf = _safe_vec3(cf.get("right")) if isinstance(cf, dict) else None
+    lpos = _safe_vec3(fp.get("left")) if isinstance(fp, dict) else None
+    rpos = _safe_vec3(fp.get("right")) if isinstance(fp, dict) else None
+    if lf:
+        out["fz_left"] = lf[2]
+        out["fxy_left"] = _norm2(lf[:2])
+    if rf:
+        out["fz_right"] = rf[2]
+        out["fxy_right"] = _norm2(rf[:2])
+    if lpos:
+        out["foot_lx"] = lpos[0]; out["foot_ly"] = lpos[1]; out["foot_lz"] = lpos[2]
+    if rpos:
+        out["foot_rx"] = rpos[0]; out["foot_ry"] = rpos[1]; out["foot_rz"] = rpos[2]
+    if lpos and rpos:
+        out["foot_dy"] = abs(lpos[1] - rpos[1])  # lateral gap
+    return out
+
@@ -155,6 +192,10 @@ def run(
             obs = adapt_obs(raw_obs)
             joint_commands = controller.step(obs)
             joint_commands = normalize_joint_commands(joint_commands)
@@
-            s = {"t": float(obs.t), "status": "RUN"}
+            s = {"t": float(obs.t), "status": "RUN"}
             qe = _q_err_rms(obs, controller)
             if qe is not None:
                 s["q_err_rms"] = float(qe)
@@
             if hasattr(obs, "base_position") and obs.base_position is not None:
                 try:
                     s["base_z"] = float(obs.base_position[2])
                 except Exception:
                     pass
+
+            # contact / feet (raw_obs based)
+            s.update(_extract_contact_and_feet(raw_obs))
+
             if tau_clip_frac is not None:
                 s["tau_clip_frac"] = float(tau_clip_frac)
             if tau_clip_max_ratio is not None:
                 s["tau_clip_max_ratio"] = float(tau_clip_max_ratio)
             samples.append(s)
```

---

### 2) `src/ext_metrics.py`（荷重偏り・滑りを集計）

```diff
--- a/src/ext_metrics.py
+++ b/src/ext_metrics.py
@@ -1,6 +1,7 @@
 from __future__ import annotations
 from typing import Any, Dict, List, Optional
 import math
 
@@ -15,6 +16,71 @@ def compute_metrics(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
     metrics: Dict[str, Any] = {}
 
+    # --- contact / slip metrics ---
+    fz_l = []
+    fz_r = []
+    fxy_l = []
+    fxy_r = []
+    foot_dy = []
+    slip_l = []
+    slip_r = []
+    last_l = None
+    last_r = None
+    for s in samples or []:
+        if not isinstance(s, dict):
+            continue
+        if "fz_left" in s:
+            try: fz_l.append(float(s["fz_left"]))
+            except Exception: pass
+        if "fz_right" in s:
+            try: fz_r.append(float(s["fz_right"]))
+            except Exception: pass
+        if "fxy_left" in s:
+            try: fxy_l.append(float(s["fxy_left"]))
+            except Exception: pass
+        if "fxy_right" in s:
+            try: fxy_r.append(float(s["fxy_right"]))
+            except Exception: pass
+        if "foot_dy" in s:
+            try: foot_dy.append(float(s["foot_dy"]))
+            except Exception: pass
+
+        # slip: step-to-step planar displacement
+        if "foot_lx" in s and "foot_ly" in s:
+            try:
+                cur = (float(s["foot_lx"]), float(s["foot_ly"]))
+                if last_l is not None:
+                    dx = cur[0]-last_l[0]; dy = cur[1]-last_l[1]
+                    slip_l.append(math.sqrt(dx*dx+dy*dy))
+                last_l = cur
+            except Exception:
+                pass
+        if "foot_rx" in s and "foot_ry" in s:
+            try:
+                cur = (float(s["foot_rx"]), float(s["foot_ry"]))
+                if last_r is not None:
+                    dx = cur[0]-last_r[0]; dy = cur[1]-last_r[1]
+                    slip_r.append(math.sqrt(dx*dx+dy*dy))
+                last_r = cur
+            except Exception:
+                pass
+
+    if fz_l and fz_r:
+        # load balance proxy: min(mean)/max(mean)
+        ml = sum(fz_l)/len(fz_l)
+        mr = sum(fz_r)/len(fz_r)
+        denom = max(1e-6, max(abs(ml), abs(mr)))
+        metrics["fz_balance_ratio"] = float(min(abs(ml), abs(mr)) / denom)
+        metrics["fz_left_mean"] = float(ml)
+        metrics["fz_right_mean"] = float(mr)
+    if fxy_l:
+        metrics["fxy_left_mean"] = float(sum(fxy_l)/len(fxy_l))
+        metrics["fxy_left_max"] = float(max(fxy_l))
+    if fxy_r:
+        metrics["fxy_right_mean"] = float(sum(fxy_r)/len(fxy_r))
+        metrics["fxy_right_max"] = float(max(fxy_r))
+    if foot_dy:
+        metrics["foot_dy_mean"] = float(sum(foot_dy)/len(foot_dy))
+        metrics["foot_dy_min"] = float(min(foot_dy))
+    if slip_l:
+        metrics["slip_left_mean"] = float(sum(slip_l)/len(slip_l))
+        metrics["slip_left_max"] = float(max(slip_l))
+    if slip_r:
+        metrics["slip_right_mean"] = float(sum(slip_r)/len(slip_r))
+        metrics["slip_right_max"] = float(max(slip_r))
+
     # --- Tracking / posture metrics (optional) ---
     qerrs = []
     rolls = []
     pitches = []
     base_zs = []
```

---

### 3) `scripts/compare_runs.py`（表示列を追加＋sort-by slip）

（必要最低限だけ。まず表示できればOK）

```diff
--- a/scripts/compare_runs.py
+++ b/scripts/compare_runs.py
@@ -18,6 +18,13 @@ class RunSummary:
     q_err_rms_mean: Optional[float]
     q_err_rms_min: Optional[float]
     roll_abs_max: Optional[float]
     pitch_abs_max: Optional[float]
+    fz_balance_ratio: Optional[float]
+    fxy_left_max: Optional[float]
+    fxy_right_max: Optional[float]
+    slip_left_mean: Optional[float]
+    slip_right_mean: Optional[float]
+    foot_dy_min: Optional[float]
     survival_time: Optional[float]
@@
 def _summarize_run(path: str) -> RunSummary:
@@
+    fz_bal = _safe_get(metrics, ["fz_balance_ratio"])
+    fxy_lm = _safe_get(metrics, ["fxy_left_max"])
+    fxy_rm = _safe_get(metrics, ["fxy_right_max"])
+    slip_lm = _safe_get(metrics, ["slip_left_mean"])
+    slip_rm = _safe_get(metrics, ["slip_right_mean"])
+    fdy_min = _safe_get(metrics, ["foot_dy_min"])
@@
     return RunSummary(
@@
         roll_abs_max=roll_abs_max,
         pitch_abs_max=pitch_abs_max,
+        fz_balance_ratio=float(fz_bal) if fz_bal is not None else None,
+        fxy_left_max=float(fxy_lm) if fxy_lm is not None else None,
+        fxy_right_max=float(fxy_rm) if fxy_rm is not None else None,
+        slip_left_mean=float(slip_lm) if slip_lm is not None else None,
+        slip_right_mean=float(slip_rm) if slip_rm is not None else None,
+        foot_dy_min=float(fdy_min) if fdy_min is not None else None,
         survival_time=float(st) if st is not None else None,
@@
 def _print_table(...):
     headers = [
@@
         "roll_max",
         "pitch_max",
+        "fz_bal",
+        "fxyLmx",
+        "fxyRmx",
+        "slipL",
+        "slipR",
+        "dy_min",
         "survival_s",
@@
             _fmt(r.roll_abs_max, 3),
             _fmt(r.pitch_abs_max, 3),
+            _fmt(r.fz_balance_ratio, 3),
+            _fmt(r.fxy_left_max, 3),
+            _fmt(r.fxy_right_max, 3),
+            _fmt(r.slip_left_mean, 4),
+            _fmt(r.slip_right_mean, 4),
+            _fmt(r.foot_dy_min, 3),
             _fmt(r.survival_time, 3),
```

---

# すぐ回すコマンド（原因特定用：ベスト近傍固定で 3秒）

今のベスト近傍で固定し、**接触/滑りの列が埋まるか**を見ます。

```bash
cd /workspace/hunter

python3 scripts/sweep_tuning.py \
  --mode grid --grid-sample spread \
  --trials 4 --repeats 1 --no-gui --duration 3 \
  --control-dt 0.01 --settle 300 \
  --warmup 999 --blend 0.0 --grav 0 \
  --stance crouch_pos \
  --crouch-knee 0.70 --crouch-ankle 0.30 \
  --pos-kp 120,240 --pos-kd 1.0,2.0

python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --limit 10
```

---

## これで次の分岐が確定します

* `slipL/slipR` が増える + `fxy_max` が大きい → **摩擦/足裏/接触**が本命
  → 次は PyBullet の **lateralFriction**（足リンクのみ）を上げる sweep（0.8/1.0/1.2/1.5）
* `fz_balance_ratio` が低い（片足荷重）→ **BASE_HEIGHT or crouch が合ってない**
  → 次は `BASE_HEIGHT` を sweep（0.62/0.65/0.68 など）
* `dy_min` が小さくなる（足幅が詰む）→ **初期姿勢/IK/足配置**問題
  → stance の hip roll（l1/r1）を広げる方向へ

---

ここまで来ると「何を動かせば 1秒壁を越えるか」がかなり確実に決まります。
もしよければ、あなたのベスト run（0.891s）の新しい `compare_runs` 出力で `fz_bal / slip / fxy` を貼ってください。次の sweep（摩擦 or BASE_HEIGHT or 足幅）を“当てに行く設定”で一発で出します。

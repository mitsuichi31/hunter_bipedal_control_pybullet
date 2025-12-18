この結果、めちゃくちゃ重要です。
**“摩擦が足りない”以前に、足（left/right）として接触が取れてない**のがほぼ確定です。

* `fz_left_mean=fz_right_mean=0.0`（足接触力ゼロ）
* `foot_dy_min ≈ 3e-5`（足幅がほぼゼロ＝左右の足が寄ってる/交差してる）
* それでも `slip_mean` が出てる（= 足位置は動いてる）
  → **足が地面に安定接触していない or 接触しているリンクが foot ではない**（スネ/膝/胴体が先に当たってる等）

ここから先の次ステップは、「足リンクでの接触」を正しく拾えるようにして、**接触しているリンク名の内訳**をログに出すのが最短です。
（今の `contact_forces["left"/"right"]` が “footリンク限定” なら、ゼロになるのは説明がつきます）

---

# 次ステップ：接触しているリンクを特定する（link-contact histogram をログ化）

## ゴール

* 「いま地面に当たってるのはどのリンクか？」を run JSON に残す
* そのうえで

  * 足リンクが当たってない → **BASE_HEIGHT/姿勢/足幅（hip roll）/crouchの見直し**
  * 足リンクが当たってるのに force=0 → **接触取得ロジックのバグ** or **リンク名 mismatch**

---

# パッチ（unified diff）

## 1) `src/simulation_env.py` に “リンク別接触集計” API を追加

（既存に近い関数があるかもですが、確実に使える形で追加）

```diff
--- a/src/simulation_env.py
+++ b/src/simulation_env.py
@@ -1,6 +1,7 @@
 import pybullet as p
 import numpy as np
 from typing import Any, Dict, List, Optional, Tuple
+from collections import defaultdict
 
@@
 class HunterSimulation:
@@
     def get_observations(self) -> Dict[str, Any]:
         # existing
         obs = {
             # ...
         }
         return obs
+
+    def get_contact_link_histogram(self) -> Dict[str, int]:
+        """
+        Return a histogram of robot link names that are in contact with the ground (or any other body).
+        Keys are link names (or 'base' for -1), values are contact point counts.
+        """
+        if self.robot_id is None:
+            return {}
+        pts = p.getContactPoints(bodyA=self.robot_id)
+        if not pts:
+            return {}
+        hist = defaultdict(int)
+        for cp in pts:
+            # pybullet contact tuple: linkIndexA at index 3
+            link_idx = cp[3]
+            if link_idx == -1:
+                name = "base"
+            else:
+                name = self.joint_info_by_index.get(link_idx, {}).get("link_name")
+                if not name:
+                    # fallback: string-ify index
+                    name = f"link_{link_idx}"
+            hist[name] += 1
+        return dict(hist)
```

> ここで使っている `self.joint_info_by_index` がもし無い場合：
> `_build_joint_info()` で作っている joint/link 情報に合わせて辞書名を調整してください。
> （あなたのリポジトリだと `joint_dict` / `controllable_joints` があるので、インデックス→link名のマップはどこかにあります。無ければ `p.getJointInfo(robot_id, i)[12]` で linkName が取れます。）

「マップが無い」場合の安全な版（遅いけど確実）に置き換え版：

```diff
+            else:
+                try:
+                    ji = p.getJointInfo(self.robot_id, link_idx)
+                    # linkName is index 12, bytes -> str
+                    name = ji[12].decode("utf-8") if isinstance(ji[12], (bytes, bytearray)) else str(ji[12])
+                except Exception:
+                    name = f"link_{link_idx}"
```

---

## 2) `src/ext_runner.py` で histogram を samples に追加（低頻度でOK）

毎サンプルは重いので、**control updateごと**に上位数個だけ入れます。

```diff
--- a/src/ext_runner.py
+++ b/src/ext_runner.py
@@ -155,6 +155,7 @@ def run(
         if (sim_time - last_control) >= control_dt:
@@
             sim.apply_hybrid_command(joint_commands)
 
             s = {"t": float(obs.t), "status": "RUN"}
@@
             s.update(_extract_contact_and_feet(raw_obs))
+
+            # Link contact histogram (top-K), for debugging "not touching with feet"
+            try:
+                hist = sim.get_contact_link_histogram()
+                if hist:
+                    top = sorted(hist.items(), key=lambda kv: kv[1], reverse=True)[:6]
+                    # store as compact "name:count" list
+                    s["contact_links_top"] = [f"{k}:{v}" for k, v in top]
+            except Exception:
+                pass
 
             samples.append(s)
```

---

## 3) `src/ext_metrics.py` で “foot接触がある割合” を出す

（リンク名に foot が含まれてるかで判定。命名は後で調整可能）

```diff
--- a/src/ext_metrics.py
+++ b/src/ext_metrics.py
@@ -16,6 +16,28 @@ def compute_metrics(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
     metrics: Dict[str, Any] = {}
 
+    # --- contact link debug ---
+    link_samples = 0
+    foot_contact_hits = 0
+    nonfoot_top = {}
+    for s in samples or []:
+        if not isinstance(s, dict):
+            continue
+        top = s.get("contact_links_top")
+        if not top:
+            continue
+        link_samples += 1
+        joined = " ".join([str(x) for x in top]).lower()
+        if ("foot" in joined) or ("ankle" in joined):
+            foot_contact_hits += 1
+        else:
+            # count the top-1 nonfoot link
+            try:
+                k = str(top[0]).split(":")[0]
+                nonfoot_top[k] = nonfoot_top.get(k, 0) + 1
+            except Exception:
+                pass
+    if link_samples > 0:
+        metrics["foot_contact_hit_rate"] = float(foot_contact_hits) / float(link_samples)
+        if nonfoot_top:
+            # store most common nonfoot top contact
+            k = sorted(nonfoot_top.items(), key=lambda kv: kv[1], reverse=True)[0][0]
+            metrics["nonfoot_contact_top1"] = k
```

---

## 4) `scripts/compare_runs.py` に表示（foot_contact_hit_rate, nonfoot_contact_top1）

```diff
--- a/scripts/compare_runs.py
+++ b/scripts/compare_runs.py
@@ -18,6 +18,8 @@ class RunSummary:
     slip_right_mean: Optional[float]
     foot_dy_min: Optional[float]
+    foot_contact_hit_rate: Optional[float]
+    nonfoot_contact_top1: Optional[str]
     survival_time: Optional[float]
@@
 def _summarize_run(path: str) -> RunSummary:
@@
+    foot_hit = _safe_get(metrics, ["foot_contact_hit_rate"])
+    nonfoot = _safe_get(metrics, ["nonfoot_contact_top1"])
@@
     return RunSummary(
@@
         foot_dy_min=float(fdy_min) if fdy_min is not None else None,
+        foot_contact_hit_rate=float(foot_hit) if foot_hit is not None else None,
+        nonfoot_contact_top1=str(nonfoot) if nonfoot is not None else None,
         survival_time=float(st) if st is not None else None,
@@
 def _print_table(...):
     headers = [
@@
         "dy_min",
+        "foot_hit",
+        "top_contact",
         "survival_s",
@@
             _fmt(r.foot_dy_min, 3),
+            _fmt(r.foot_contact_hit_rate, 2),
+            (r.nonfoot_contact_top1 or "-"),
             _fmt(r.survival_time, 3),
```

---

# すぐ回すコマンド（再現を固定して “何が当たってるか” 見る）

あなたのベスト近傍固定でOK。まず 1本でもいいです。

```bash
cd /workspace/hunter
python3 scripts/sweep_tuning.py \
  --mode grid --trials 1 --repeats 1 --no-gui --duration 3 \
  --control-dt 0.01 --settle 300 --warmup 999 --blend 0.0 --grav 0 \
  --stance crouch_pos --crouch-knee 0.70 --crouch-ankle 0.30 \
  --pos-kp 120 --pos-kd 1.0 \
  --kp 20 --kd 4.5 --tau 150
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by mtime --limit 5
```

---

# この結果で次のアクションが確定します

### ケースA：`foot_contact_hit_rate` が低い（例 0.0〜0.2）＋ `top_contact` が shin/thigh/base など

→ 足が接地してない/足以外が当たってる
**次：BASE_HEIGHT sweep** が最優先です（腰が高すぎるか低すぎる）

**推奨 sweep（最小）**

* BASE_HEIGHT を 3点で振る（例：今が 0.68 付近なら 0.62/0.65/0.68）

### ケースB：foot_hit_rate が高いのに fz が 0

→ 取得ロジックの問題 or contact_forces の定義が “別の計測”
**次：contact_forces の計算箇所（simulation_env.py: get_contact_forces）を修正**（footリンク index の取り違えが典型）

---

今の観測（fz=0, dy_min≈0）は「足が地面にちゃんと乗ってない」側が濃厚なので、**まずは“どのリンクが接触してるか”**を出すのが一番早いです。

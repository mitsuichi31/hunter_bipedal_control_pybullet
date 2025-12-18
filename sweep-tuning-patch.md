了解。今回の結果（DONE 0/12・tilt 100%）よりも先に、あなたが指摘した **grid downsample の偏り**を直さないと、探索が“本当に振れているか”すら保証できません。
次はまず **sweep_tuning.py の downsample を「偏りにくい選び方」に変更**しましょう。

以下に **パッチ（unified diff）**を出します（grid の間引きを「ハッシュで並べ替え→等間隔サンプル」に変更して、`--trials` が少なくても各パラメータが偏りにくくなります）。

---

## scripts/sweep_tuning.py の差分パッチ

```diff
--- a/scripts/sweep_tuning.py
+++ b/scripts/sweep_tuning.py
@@ -1,10 +1,12 @@
 #!/usr/bin/env python3
 from __future__ import annotations
 
 import argparse
 import itertools
 import json
+import hashlib
 import os
 import random
 from dataclasses import dataclass
 from typing import Any, Dict, List, Optional
 
@@ -140,6 +142,7 @@
     ap.add_argument("--no-gui", action="store_true", help="Force --no-gui for main_simulation.py")
     ap.add_argument("--dry-run", action="store_true", help="Print planned trials but do not run")
     ap.add_argument("--warmup", default="1.0", help="Comma-separated warmup_seconds candidates (two_stage)")
     ap.add_argument("--blend", default="0.0", help="Comma-separated blend_seconds candidates (two_stage)")
     ap.add_argument("--grav", default="0,1", help="Comma-separated use_gravity_comp candidates: 0 or 1 (two_stage)")
     ap.add_argument("--grav-scale", default="1.0", help="Comma-separated gravity_scale candidates (two_stage)")
     ap.add_argument("--kd-blend", default="1.0", help="Comma-separated kd_blend_factor candidates (two_stage)")
+    ap.add_argument("--grid-sample", default="spread", choices=["spread", "random"], help="How to downsample grid when --trials < full grid size")
 
@@ -176,6 +179,44 @@
     grav_scales = _mk_grid(parse_floats(args.grav_scale))
     kd_blends = _mk_grid(parse_floats(args.kd_blend))
 
+    def _stable_hash(x: Any) -> str:
+        # Stable, deterministic ordering independent of Python's hash randomization.
+        b = repr(x).encode("utf-8")
+        return hashlib.md5(b).hexdigest()
+
+    def _downsample_grid(all_params: List[Any], k: int) -> List[Any]:
+        """
+        Downsample a full-factorial grid to k points with less bias than naive step slicing.
+        - spread: hash-sort then take evenly spaced indices (deterministic)
+        - random: deterministic shuffle by seed then take first k
+        """
+        if k <= 0:
+            return []
+        if len(all_params) <= k:
+            return all_params
+
+        if args.grid_sample == "random":
+            rng = random.Random(args.seed)
+            tmp = list(all_params)
+            rng.shuffle(tmp)
+            return tmp[:k]
+
+        # spread (default): hash-sort then evenly pick indices
+        tmp = sorted(all_params, key=_stable_hash)
+        if k == 1:
+            return [tmp[len(tmp) // 2]]
+        # Evenly spaced indices across [0, n-1]
+        n = len(tmp)
+        picks = []
+        for i in range(k):
+            idx = int(round(i * (n - 1) / (k - 1)))
+            picks.append(tmp[idx])
+        # De-dup in case of rounding collisions (rare when k<<n); fill from front if needed
+        seen = set()
+        out = []
+        for p in picks:
+            if p not in seen:
+                seen.add(p)
+                out.append(p)
+        for p in tmp:
+            if len(out) >= k:
+                break
+            if p not in seen:
+                seen.add(p)
+                out.append(p)
+        return out
+
     planned_params: List[Dict[str, Any]] = []
 
     if args.mode == "grid":
         # Keep grid manageable by subsampling if too large
         all_params = list(itertools.product(kps, kds, taus, dts, settles, warmups, blends, grav_flags, grav_scales, kd_blends))
         max_grid = max(1, args.trials)
         if len(all_params) > max_grid:
-            # deterministic subsampling
-            step = max(1, len(all_params) // max_grid)
-            all_params = all_params[::step][:max_grid]
+            all_params = _downsample_grid(all_params, max_grid)
         for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend in all_params:
             planned_params.append(
                 {
                     "kp": kp,
                     "kd": kd,
@@ -220,6 +267,26 @@
                     "kd_blend_factor": float(kd_blend),
                 }
             )
+
+        # Print actual coverage (helps detect accidental "fixed" dimensions)
+        try:
+            used = {
+                "kp": sorted({p["kp"] for p in planned_params}),
+                "kd": sorted({p["kd"] for p in planned_params}),
+                "tau_limit": sorted({p["tau_limit"] for p in planned_params}),
+                "settle_steps": sorted({p["settle_steps"] for p in planned_params}),
+                "warmup_seconds": sorted({p["warmup_seconds"] for p in planned_params}),
+                "blend_seconds": sorted({p["blend_seconds"] for p in planned_params}),
+                "use_gravity_comp": sorted({p["use_gravity_comp"] for p in planned_params}),
+                "gravity_scale": sorted({p["gravity_scale"] for p in planned_params}),
+                "kd_blend_factor": sorted({p["kd_blend_factor"] for p in planned_params}),
+            }
+            print("[grid] actual coverage:", used)
+        except Exception:
+            pass
     else:
         random.seed(args.seed)
         for _ in range(args.trials):
             planned_params.append(
                 {
```

---

## 使い方（次に回すコマンド）

### 1) 今回と同じ条件で、偏りにくい grid downsample を確認

```bash
cd /workspace/hunter
python3 scripts/sweep_tuning.py \
  --mode grid --trials 12 --repeats 1 --no-gui \
  --grid-sample spread \
  --control-dt 0.01 \
  --settle 0,300 \
  --warmup 0.5,1.0 \
  --blend 0.2,0.4 \
  --grav 1 \
  --grav-scale 0.8,1.0,1.2 \
  --kp 20,30,40 \
  --kd 1.5,3.0 \
  --kd-blend 2.0,3.0 \
  --tau 60,90,120,150
```

実行ログに `[grid] actual coverage:` が出るので、**本当に各次元が振れているか**が一目で分かります。

### 2) もし「少ない trials で多次元」は辛いなら、random の方が現実的

```bash
python3 scripts/sweep_tuning.py \
  --mode random --trials 40 --repeats 1 --no-gui \
  --seed 0 \
  --control-dt 0.01 \
  --settle 0,300 \
  --warmup 0.5,1.0 \
  --blend 0.2,0.4 \
  --grav 1 \
  --grav-scale 0.8,1.0,1.2 \
  --kp 20,30,40 \
  --kd 1.5,3.0 \
  --kd-blend 2.0,3.0 \
  --tau 60,90,120,150
```

---

## その上で（tilt 100% への即効の次手）

grid 偏りを直しても tilt が 100% 続くなら、次は **“安定化側の探索”に寄せる**のが早いです：

* `kd` を広げる（例：`--kd 3.0,4.5,6.0`）
* `kd_blend_factor` を上げる（例：`--kd-blend 3.0,4.0`）
* `tau_limit` を上げる（例：`--tau 120,150,180`）

…を優先して回すのが効きます。

必要なら、このパッチ適用後のあなたの `sweep_summary.json` を前提に「次の絞り込み sweep（パラメータ最小構成）」もこちらで提案します。


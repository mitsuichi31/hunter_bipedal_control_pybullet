```diff
--- a/scripts/compare_runs.py
+++ b/scripts/compare_runs.py
@@ -1,9 +1,9 @@
 #!/usr/bin/env python3
 from __future__ import annotations
 
 import argparse
 import json
 import os
 from dataclasses import dataclass
 from datetime import datetime
 from typing import Any, Dict, List, Optional, Tuple
@@ -210,13 +210,13 @@
 def main() -> int:
     ap = argparse.ArgumentParser()
     ap.add_argument("--log-dir", default="runs")
     ap.add_argument("--prefix", default="")
     ap.add_argument("--limit", type=int, default=10)
     ap.add_argument(
         "--sort-by",
         default="mtime",
-        choices=["mtime", "survival", "score", "tilt", "energy"],
+        choices=["mtime", "survival", "score", "tilt", "energy", "clip"],
         help="Sort criterion (default: newest first)",
     )
     ap.add_argument(
         "--grav",
         default="any",
@@ -270,6 +270,17 @@
     elif args.sort_by == "tilt":
         runs.sort(key=lambda r: (r.tilt_max_abs is None, (r.tilt_max_abs or 0.0), -r.mtime))
     elif args.sort_by == "energy":
         runs.sort(key=lambda r: (r.energy_abs_tau_dq is None, (r.energy_abs_tau_dq or 0.0), -r.mtime))
+    elif args.sort_by == "clip":
+        # Lower saturation is better:
+        #   1) clip_mean asc
+        #   2) clip_max asc
+        #   3) score desc (tie-break)
+        #   4) newest last key
+        runs.sort(
+            key=lambda r: (
+                r.tau_clip_frac_mean is None, (r.tau_clip_frac_mean or 1e9),
+                r.tau_clip_frac_max is None, (r.tau_clip_frac_max or 1e9),
+                r.score is None, -(r.score or 0.0),
+                -r.mtime,
+            )
+        )
 
     _print_table(runs, args.limit)
 
     best = runs[0]
     print("\nBest (by sort):", os.path.join(args.log_dir, os.path.basename(best.path)))
     return 0
```

使い方例：

```bash
cd /workspace/hunter

# 飽和が少ない順に上位を見る（同率なら score が高いものが上）
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by clip --limit 15

# grav=ON の中で飽和が少ない順
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by clip --grav on --limit 15
```

これで「**無理して立ってる（飽和だらけ）**」設定を避けて、次の探索対象を選びやすくなります。

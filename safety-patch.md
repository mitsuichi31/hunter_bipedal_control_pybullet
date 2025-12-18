了解。ここまでのログは「次の切り分け」に十分で、結論はこうです：

* **q_err_rms が小さい＝追従はできている**
* **clip/飽和でもない**
* **abort は tilt（今回のベストは roll=0.703 rad 付近で切られている）**
* **survival は 0.88〜0.89s で頭打ち**

つまり今は **「倒れている」より先に「安全判定で切られている」**可能性が高いです。
少なくとも「回復できるか」を確かめるには、安全閾値を一時的に緩めて観測する価値が大きいです。

次のステップは（あなたが前に提案してくれた通り）：

> **safety.max_roll/max_pitch を“観測用”に緩めて、1秒以降に回復傾向があるかを見る**

これで分岐できます：

* 緩めたら 2〜10秒まで生存が伸びる → **回復可能だが閾値が厳しすぎる / 0.7rad近辺で一瞬振れる**
  → 次は「ロールのピークを抑える」制御（kd↑、blend↑、crouch調整、足幅など）へ
* 緩めても 1秒前後で転倒が進行する → **本当に倒れている**
  → 次は「姿勢自体の安定化」（BASE_HEIGHT、摩擦、足幅、姿勢）へ

---

# 次ステップパッチ：safety 閾値を sweep で上書きできるようにする

（既存 `config/agent_tuning.yaml` の safety を尊重しつつ、指定時だけ上書き）

## 1) `scripts/sweep_tuning.py` に `--max-roll`, `--max-pitch` を追加

```diff
--- a/scripts/sweep_tuning.py
+++ b/scripts/sweep_tuning.py
@@ -150,6 +150,8 @@
     ap.add_argument("--stance", default="standing", help="Comma-separated stance candidates: standing,crouch_pos,crouch_neg")
     ap.add_argument("--crouch-knee", default=None, help="Comma-separated crouch knee magnitudes (used when stance is crouch_*)")
     ap.add_argument("--crouch-ankle", default=None, help="Comma-separated crouch ankle magnitudes (used when stance is crouch_*)")
+    ap.add_argument("--max-roll", default=None, help="Comma-separated safety max_roll (rad) overrides")
+    ap.add_argument("--max-pitch", default=None, help="Comma-separated safety max_pitch (rad) overrides")
     ap.add_argument("--grid-sample", default="spread", choices=["spread", "random"], help="How to downsample grid when --trials < full grid size")
@@ -199,6 +201,8 @@
     crouch_knees = _mk_grid(parse_floats(args.crouch_knee)) if args.crouch_knee is not None else None
     crouch_ankles = _mk_grid(parse_floats(args.crouch_ankle)) if args.crouch_ankle is not None else None
+    max_rolls = _mk_grid(parse_floats(args.max_roll)) if args.max_roll is not None else None
+    max_pitches = _mk_grid(parse_floats(args.max_pitch)) if args.max_pitch is not None else None
@@ -212,6 +216,8 @@
         _ck = crouch_knees if crouch_knees is not None else [None]
         _ca = crouch_ankles if crouch_ankles is not None else [None]
+        _mr = max_rolls if max_rolls is not None else [None]
+        _mp = max_pitches if max_pitches is not None else [None]
         all_params = list(
             itertools.product(
-                kps, kds, taus, dts, settles, warmups, blends, grav_flags, grav_scales, kd_blends, _pos_kps, _pos_kds, stances, _ck, _ca
+                kps, kds, taus, dts, settles, warmups, blends, grav_flags, grav_scales, kd_blends,
+                _pos_kps, _pos_kds, stances, _ck, _ca, _mr, _mp
             )
         )
@@ -221,7 +227,7 @@
         if len(all_params) > max_grid:
             all_params = _downsample_grid(all_params, max_grid)
-        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend, pos_kp, pos_kd, stance, ck, ca in all_params:
+        for kp, kd, tau, dt, settle, warmup, blend, grav, gscale, kd_blend, pos_kp, pos_kd, stance, ck, ca, mr, mp in all_params:
             planned_params.append(
                 {
@@ -238,6 +244,8 @@
                     "stance": str(stance),
                     "crouch_knee": float(ck) if ck is not None else None,
                     "crouch_ankle": float(ca) if ca is not None else None,
+                    "max_roll": float(mr) if mr is not None else None,
+                    "max_pitch": float(mp) if mp is not None else None,
                 }
             )
@@ -255,6 +263,8 @@
                     "stance": str(random.choice(stances)),
                     "crouch_knee": float(random.choice(crouch_knees)) if crouch_knees is not None else None,
                     "crouch_ankle": float(random.choice(crouch_ankles)) if crouch_ankles is not None else None,
+                    "max_roll": float(random.choice(max_rolls)) if max_rolls is not None else None,
+                    "max_pitch": float(random.choice(max_pitches)) if max_pitches is not None else None,
                 }
             )
@@ -310,6 +320,18 @@
         # safety overrides (optional)
-        cfg_safety = dict(cfg.get("safety") or {})
+        cfg_safety = dict(cfg.get("safety") or {})
+        if p.get("max_roll", None) is not None:
+            cfg_safety["max_roll"] = float(p["max_roll"])
+        if p.get("max_pitch", None) is not None:
+            cfg_safety["max_pitch"] = float(p["max_pitch"])
+        cfg["safety"] = cfg_safety
```

> ※ best-out への反映も同様に入れたい場合は、best_cfg の safety ブロックにも同じ上書きを入れます（必要なら追記版 diff を出しますが、まずは観測が目的なので上書きは trial 側だけで十分です）。

---

# 次に回す “観測用” コマンド（最短）

いまのベスト近傍を固定して、**安全閾値だけ振る**のが一番綺麗に切り分けできます。

あなたの今回ベスト（crouch_pos, cknee=0.70, cankle=0.30, pos_kp=120, pos_kd=1.0）で固定：

```bash
cd /workspace/hunter

python3 scripts/sweep_tuning.py \
  --mode grid --grid-sample spread \
  --trials 6 --repeats 1 --no-gui \
  --duration 3 \
  --control-dt 0.01 --settle 300 \
  --warmup 999 --blend 0.0 --grav 0 \
  --stance crouch_pos \
  --crouch-knee 0.70 --crouch-ankle 0.30 \
  --pos-kp 120 --pos-kd 1.0 \
  --max-roll 0.70,0.85,1.00 \
  --max-pitch 0.70,0.85,1.00
```

※ `--duration` が sweep に無ければ省略（runner側のデフォルトでOK）

見る：

```bash
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --limit 20
python3 scripts/analyze_abort_reasons.py --summary runs/sweep_summary.json --limit 12 --min-trials 1
```

---

## 期待する読み取り

* max_roll/max_pitch を 0.85〜1.0 にした瞬間に **survival が 1s を超える**なら、

  * “本当は少し揺れるが戻せる”状態
  * 次は **rollピークの抑制**（kd↑、kd_blend_factor↑、blend_seconds↑、cankle微調整）で閾値内に収めに行く
* それでも伸びないなら、次は **BASE_HEIGHT sweep**（腰高/低すぎ問題）へ

---

あなたのログだと、切られている roll が **0.703rad**でほぼ閾値ちょうどっぽいので、この観測はかなり当たる可能性が高いです。

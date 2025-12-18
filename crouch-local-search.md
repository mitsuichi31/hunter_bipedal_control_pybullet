了解。ここまでの結果から、次の一手はかなり明確です。

* **crouch は効く**（standing 0.55s → crouch 0.88s）
* ただし **深さ sweep 24本でも 0.88s を超えない**
* そして決定的に、今回 **cankle=0.30 を試していない**（テンプレ相当が最強の可能性が高い）
* さらに、浅い crouch は settle 即死（updates=0）→「浅い＝腰高」が不安定

なので次は「広く振る」より、**テンプレ周辺を局所探索**して 1秒壁を割りにいくのが最短です。

---

## 次に回すコマンド（局所探索：cankle=0.30 を必ず含める）

まずは position-only のまま、**0.30 を中心に ankle を細かく**、knee もテンプレ近傍で振ります。

```bash
cd /workspace/hunter

python3 scripts/sweep_tuning.py \
  --mode grid --grid-sample spread \
  --trials 24 --repeats 1 --no-gui \
  --control-dt 0.01 --settle 300 \
  --warmup 999 --blend 0.0 --grav 0 \
  --stance crouch_pos,crouch_neg \
  --crouch-knee 0.55,0.60,0.65,0.70 \
  --crouch-ankle 0.25,0.30,0.35 \
  --pos-kp 120,240 \
  --pos-kd 1.0,2.0 \
  --kp 20 --kd 4.5 --tau 150
```

見るコマンドはいつも通り：

```bash
python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --limit 20
python3 scripts/analyze_abort_reasons.py --summary runs/sweep_summary.json --limit 12 --min-trials 1
```

---

## それでも 1秒に届かない場合の「次の次」

ここまでで **追従はOKなのに tilt で死ぬ**が続くので、次は **“安全閾値の観測緩和”**で「回復できているのに切られている」かを切り分けます。

### 観測用（比較目的ではなく切り分け用）

* `safety.max_pitch/max_roll` を一時的に 2倍程度にして、**倒れずに復帰する挙動があるか**を見る
* もし復帰するなら、コントローラ側（特に damping/足裏/腰高）に修正余地がある
* 復帰しないなら、単純に姿勢が崩れて戻せていない

（必要なら `scripts/sweep_tuning.py` に `--max-roll/--max-pitch` 追加パッチを一括で出します）

---

## 小さな観察ポイント（今回のログからの推測）

* ベストが `cknee=0.75` でも伸びない → **深くし過ぎると足首/接触が厳しい可能性**
* 旧ベスト（テンプレ相当 0.6/0.3）が強い → **“足首0.30付近”がキー**の可能性が高い

---

次は上の局所探索を回して、上位3本の

* stance / cknee / cankle / pos_kp / pos_kd
* survival / roll/pitch_max / base_z_min
  を見れば、「1秒壁に足りないのが ankle 側か knee 側か」がほぼ決まります。

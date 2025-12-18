τ_limit / kp / kd の「DONE 出し」用最小 sweep

```bash
# (inside Docker) DONEを出しに行く最小sweep（tau_limit主軸 + 減衰/切替を少し振る）
cd /workspace/hunter

python3 scripts/sweep_tuning.py \
  --mode grid \
  --trials 12 \
  --repeats 1 \
  --no-gui \
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

# 結果チェック：まずはDONE/長生き/低飽和の上位を確認
python3 scripts/compare_runs.py \
  --log-dir runs \
  --prefix standing_pd_ext \
  --grav on \
  --sort-by clip \
  --limit 20

# DONEが混ざってきたら scoreでも見る
python3 scripts/compare_runs.py \
  --log-dir runs \
  --prefix standing_pd_ext \
  --grav on \
  --sort-by score \
  --limit 20

# abort理由の偏りを見る（次の手が決まる）
python3 scripts/analyze_abort_reasons.py --summary runs/sweep_summary.json --limit 12 --min-trials 3
```

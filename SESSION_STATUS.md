# Session Status (feature/trial-work-by-using-agent)

Date: 2025-12-18

このセッションは **standing-pd-ext 系の拡張 / sweep・比較ツール整備 / 「なぜ立てないか」の切り分け** を主目的として進めました。
（ローカルに PyBullet が無いため、検証は Docker コンテナ `hunter-simulation` で実施）

---

## What we changed (code / tooling)

### 1) stance（新しい姿勢ターゲット）導入 + crouch 量の可変化
- `src/robot_constants.py`
  - stance 定義を追加（`standing`, `crouch_pos`, `crouch_neg`）
  - `get_stance_config()` 追加
- `src/ext_standing_ref.py`
  - `stance_q_ref(stance, crouch_knee, crouch_ankle)` を導入（crouch量の上書き対応）
- `src/main_simulation.py`
  - `controller.stance` と `controller.crouch_knee/crouch_ankle` を読み、reset姿勢と `q_ref` を切替

### 2) sweep/compare の拡張（実験を回しやすくする）
- `scripts/sweep_tuning.py`
  - sweep 対象を拡張: `--stance`, `--crouch-knee`, `--crouch-ankle`, `--max-roll`, `--max-pitch`, `--base-height`
  - trial ごとに YAML（`config/agent_tuning.yaml`）を書き換えて走らせる方式
- `scripts/compare_runs.py`
  - 表示列を増強: stance/crouch量、安全閾値、接触/滑り/足幅など（`ct_any`, `foot_hit`, `top_contact`, `dy_min` 等）
  - 追加ソート: `--sort-by slip` 等

### 3) 接触/滑り/追従の計測を追加（原因の可視化）
- `src/ext_runner.py`
  - `q_err_rms`（追従誤差RMS）を samples に記録
  - 足位置・接触力（`fz_*`, `fxy_*`, `foot_*`, `foot_dy`）を samples に記録
  - 接触リンク上位（`contact_links_top`）を samples に記録
- `src/simulation_env.py`
  - `get_contact_link_histogram()` 追加（接触しているリンク名の集計）
- `src/ext_metrics.py`
  - `q_err_rms_*`、`slip_*`、`foot_dy_min`、`contact_any_rate`、`foot_contact_hit_rate` 等を集計
  - `nonfoot_contact_top1` の表示を改善（接触がある場合に `FOOT_CONTACT` 等を入れる）

### 4) BASE_HEIGHT の sweep 対応（初期条件の切り分け）
- `config/agent_tuning.yaml`
  - `runner.base_height: null` を追加（`null` は既定の `BASE_HEIGHT` を使用）
- `src/main_simulation.py`
  - `runner.base_height` が設定されていれば初期 base height として利用し、meta に記録

---

## What we ran (Docker verification commands)

※すべて `hunter-simulation` で実行（例：`docker exec hunter-simulation bash -lc 'cd /workspace/hunter && ...'`）。

## Representative logs (runs/*.json)

（クリックで開けるようにファイル名のみ列挙。用途/再現条件のメモ付き）

- `runs/standing_pd_ext_20251218_131400.json`：現時点の代表（`base_h=0.679`, `crouch_pos 0.7/0.3`, `ct_any≈0.02`, `top_contact=FOOT_CONTACT`）
- `runs/standing_pd_ext_20251218_131208.json`：base height 近傍 sweep の `base_h=0.679`（同条件で `survival≈0.891s`）
- `runs/standing_pd_ext_20251218_131025.json`：safety 観測緩和（`max_roll=1.0`）で `survival≈0.961s` まで伸びるが、roll が閾値まで発散して abort
- `runs/standing_pd_ext_20251218_131146.json`：base height 粗 sweep の相対的ベスト（`base_h=0.65`, `survival≈0.58s`）
- `runs/standing_pd_ext_20251218_130724.json`：stance sweep のベスト例（`standing` より `crouch_pos` が改善することを確認）

## compare_runs.py snapshots (top rows)

### Latest runs (by mtime)

```
docker exec hunter-simulation bash -lc 'cd /workspace/hunter && python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by mtime --limit 8'

rank | time                | status | score  | grav | g_scale | blend | kd_blend | base_h | stance     | cknee | cankle | s_roll | s_pitch | pos_kp | pos_kd | ... | foot_hit | ct_any | top_contact  | base_z_min | abort_reason                            | file
1    | 2025-12-18 13:14:00 | ABORT  | 0.551  | OFF  | 1.000   | 0.000 | 1.000    | 0.679  | crouch_pos | 0.700 | 0.300  | 0.700  | 0.700   | 120.00 | 1.00   | ... | 0.02     | 0.02   | FOOT_CONTACT | 0.552      | tilt too large roll=0.703 pitch=0.107   | standing_pd_ext_20251218_131400.json
2    | 2025-12-18 13:13:48 | ABORT  | 0.050  | OFF  | 1.000   | 0.000 | 1.000    | 0.690  | crouch_pos | 0.700 | 0.300  | 0.700  | 0.700   | 120.00 | 1.00   | ... | 0.00     | 0.00   | NO_CONTACT   | 0.508      | tilt too large roll=0.701 pitch=-0.242  | standing_pd_ext_20251218_131348.json
3    | 2025-12-18 13:12:08 | ABORT  | 0.551  | OFF  | 1.000   | 0.000 | 1.000    | 0.679  | crouch_pos | 0.700 | 0.300  | 0.700  | 0.700   | 120.00 | 1.00   | ... | 0.02     | 0.02   | -            | 0.552      | tilt too large roll=0.703 pitch=0.107   | standing_pd_ext_20251218_131208.json
```

### Best runs (by score)

```
docker exec hunter-simulation bash -lc 'cd /workspace/hunter && python3 scripts/compare_runs.py --log-dir runs --prefix standing_pd_ext --sort-by score --limit 8'

rank | time                | status | score | grav | ... | base_h | stance     | cknee | cankle | ... | survival_s | ct_any | top_contact  | abort_reason                            | file
1    | 2025-12-18 07:04:21 | ABORT  | 0.607 | ON   | ... | -      | -          | -     | -      | ... | 0.951      | -     | -            | tilt too large roll=-0.701 pitch=-0.393 | standing_pd_ext_20251218_070421.json
2    | 2025-12-18 13:14:00 | ABORT  | 0.551 | OFF  | ... | 0.679  | crouch_pos | 0.700 | 0.300  | ... | 0.891      | 0.02  | FOOT_CONTACT | tilt too large roll=0.703 pitch=0.107   | standing_pd_ext_20251218_131400.json
3    | 2025-12-18 13:12:08 | ABORT  | 0.551 | OFF  | ... | 0.679  | crouch_pos | 0.700 | 0.300  | ... | 0.891      | 0.02  | -            | tilt too large roll=0.703 pitch=0.107   | standing_pd_ext_20251218_131208.json
```

### A) stance sweep（`add-new-stance.md`）
- 実行:
  - `python3 scripts/sweep_tuning.py --mode grid --grid-sample spread --trials 12 --repeats 1 --no-gui --duration 3 --control-dt 0.01 --settle 300 --warmup 999 --blend 0.0 --grav 0 --stance standing,crouch_pos,crouch_neg --pos-kp 120,240 --pos-kd 1.0,2.0 --kp 20 --kd 4.5 --tau 150 --grav-scale 1.0 --kd-blend 1.0 --base-height 0.679`
- 結果（要点）:
  - `standing` はおおむね `survival≈0.54s`
  - `crouch_pos` は `survival≈0.89s` まで改善（ただし 1 秒は超えない）

### B) crouch depth sweep（`add-crouch-depth-sweep.md`）
- 実行:
  - `python3 scripts/sweep_tuning.py --mode grid --grid-sample spread --trials 24 --repeats 1 --no-gui --duration 3 --control-dt 0.01 --settle 300 --warmup 999 --blend 0.0 --grav 0 --stance crouch_pos,crouch_neg --crouch-knee 0.3,0.45,0.6,0.75 --crouch-ankle 0.15,0.25,0.35 --pos-kp 120,240 --pos-kd 1.0,2.0 --kp 20 --kd 4.5 --tau 150 --grav-scale 1.0 --kd-blend 1.0 --base-height 0.679`
- 結果（要点）:
  - ベストは `survival≈0.83s` 程度（テンプレ `0.7/0.3` を超えず）

### C) crouch local search（`crouch-local-search.md`）
- 実行:
  - `python3 scripts/sweep_tuning.py --mode grid --grid-sample spread --trials 24 --repeats 1 --no-gui --duration 3 --control-dt 0.01 --settle 300 --warmup 999 --blend 0.0 --grav 0 --stance crouch_pos,crouch_neg --crouch-knee 0.55,0.60,0.65,0.70 --crouch-ankle 0.25,0.30,0.35 --pos-kp 120,240 --pos-kd 1.0,2.0 --kp 20 --kd 4.5 --tau 150 --grav-scale 1.0 --kd-blend 1.0 --base-height 0.679`
- 結果（要点）:
  - ベストは `crouch_pos cknee=0.70 cankle=0.30 pos_kp=120 pos_kd=1.0` で `survival≈0.891s`

### D) safety 観測緩和（`safety-patch.md`）
- 実行:
  - `python3 scripts/sweep_tuning.py --mode grid --grid-sample spread --trials 9 --repeats 1 --no-gui --duration 3 --control-dt 0.01 --settle 300 --warmup 999 --blend 0.0 --grav 0 --stance crouch_pos --crouch-knee 0.70 --crouch-ankle 0.30 --pos-kp 120 --pos-kd 1.0 --kp 20 --kd 4.5 --tau 150 --grav-scale 1.0 --kd-blend 1.0 --max-roll 0.70,0.85,1.00 --max-pitch 0.70,0.85,1.00 --base-height 0.679`
- 結果（要点）:
  - `max_roll/max_pitch` を上げると survival は `0.891s → 0.921s → 0.961s` と少し伸びる
  - ただし **roll が新しい閾値まで増大して abort**（回復して安定化する兆候は無し）

### E) link-contact 観測（`log-link-contact.md`）
- 実行（1本）:
  - `python3 scripts/sweep_tuning.py --mode grid --trials 1 --repeats 1 --no-gui --duration 3 --control-dt 0.01 --settle 300 --warmup 999 --blend 0.0 --grav 0 --stance crouch_pos --crouch-knee 0.70 --crouch-ankle 0.30 --pos-kp 120 --pos-kd 1.0 --kp 20 --kd 4.5 --tau 150 --grav-scale 1.0 --kd-blend 1.0 --max-roll 0.7 --max-pitch 0.7 --base-height 0.679`
- 結果（要点）:
  - `contact_any_rate(ct_any)` はベスト付近でも `≈0.0167`（制御更新のほとんどで接触が無い）
  - `top_contact` は `FOOT_CONTACT` になる（ただし接触サンプル自体が希薄）

### F) base height sweep（`add-sweep-base-hight.md`）
- 粗 sweep:
  - `python3 scripts/sweep_tuning.py --mode grid --grid-sample spread --trials 6 --repeats 1 --no-gui --duration 3 --control-dt 0.01 --settle 300 --warmup 999 --blend 0.0 --grav 0 --stance crouch_pos --crouch-knee 0.70 --crouch-ankle 0.30 --pos-kp 120 --pos-kd 1.0 --kp 20 --kd 4.5 --tau 150 --grav-scale 1.0 --kd-blend 1.0 --max-roll 0.7 --max-pitch 0.7 --base-height 0.45,0.50,0.55,0.60,0.65,0.70`
- 近傍 sweep（canonical 0.679 周辺）:
  - `python3 scripts/sweep_tuning.py --mode grid --grid-sample spread --trials 5 --repeats 1 --no-gui --duration 3 --control-dt 0.01 --settle 300 --warmup 999 --blend 0.0 --grav 0 --stance crouch_pos --crouch-knee 0.70 --crouch-ankle 0.30 --pos-kp 120 --pos-kd 1.0 --kp 20 --kd 4.5 --tau 150 --grav-scale 1.0 --kd-blend 1.0 --max-roll 0.7 --max-pitch 0.7 --base-height 0.66,0.67,0.679,0.685,0.69`
- 結果（要点）:
  - 粗 sweep では `0.65` が相対的にマシ（`survival≈0.58s`）だが全体的に不安定
  - 近傍 sweep では **`0.679` が明確に最良**（`survival≈0.891s`、`ct_any≈0.0167`）
  - `0.679±0.01` で急に悪化し、初期高さの許容域がかなり狭い

---

## Key findings (interpretation)

1) `q_err_rms` は小さい（追従は概ねOK）  
2) それでも転倒する主因は **接触が継続して成立していない**ことが強く示唆される  
   - `ct_any(contact_any_rate)` がベストでも `~0.01–0.02` 程度  
3) `safety.max_roll/max_pitch` を緩めても「復帰して安定」はせず、**roll が発散して閾値で止まる**  
4) `foot_dy_min` が極小（左右足幅が潰れる）になる挙動があり、姿勢崩れと相関が高い疑い

---

## Current branch / working tree
- Branch: `feature/trial-work-by-using-agent`
- `config/agent_tuning.yaml` はベースラインに復帰済み（sweep は trial ごとに上書きする方式）

---

## Next steps (what to do next)

優先度順：

1) **「接触を安定させる」方向に集中**（今はほぼ空中/バウンド時間が支配的）
   - まずは「足幅が潰れる（`foot_dy_min→0`）」原因を潰す
   - 具体案:
     - `STANDING_CONFIG` / stance 内の hip roll（`leg_l1_joint/leg_r1_joint`）の見直し（足幅を確保）
     - reset 時の初期姿勢（足裏の水平度・左右対称性）の再確認
   - **次タスク**: `add-hip-roll-sweep.md`（hip roll を sweep して足幅を確保し、`ct_any`/`foot_dy_min` の改善を狙う）

2) **base height の“狭い当たり領域”を、接触率 `ct_any` 最大化で探索**
   - 指標: `ct_any`, `foot_hit`, `foot_dy_min`, `survival_s`
   - コマンド例（当たり周辺をさらに細かく）:
     - `python3 scripts/sweep_tuning.py ... --base-height 0.675,0.677,0.679,0.681,0.683`

3) 接触が安定したら、その状態で **crouch local search を再度**（1秒壁突破のための再探索）

4) その後に（必要なら）摩擦/接触パラメータや制御ゲインの本格チューニング

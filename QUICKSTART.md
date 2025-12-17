# クイックスタート（最新状態）

Hunter 二足歩行ロボットのシミュレーションを最短で試す手順です。現在の安定パスは「位置制御歩行」と「MPC+WBC 立位」です。

## 1. 環境準備（Docker 推奨）
```bash
cd docker
docker-compose up -d
# コンテナ名: hunter-simulation
```

## 2. モード別実行例（コンテナ内）
- 立位（PD, GUIなし）  
  `docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode standing --duration 10 --no-gui`
- 立位（MPC+ZMP, GUIなし）  
  `docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode standing-mpc --duration 10 --no-gui`
- 位置制御歩行（安定版）  
  `docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode walking --duration 12 --no-gui`
- WBC 立位（MPC+WBC統合）  
  `docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode wbc --duration 10 --no-gui`

GUI を使う場合は X11 設定が必要です（Linux/WSL2 なら `xhost +local:docker`、macOS は XQuartz）。

## 3. テスト
- フルハーネス: `docker exec hunter-simulation bash /workspace/hunter/scripts/test_all_modes.sh`  
  - 立位/MPC/WBC/位置制御歩行のスモーク＋歩行回帰を含む
- 単体テスト例:  
  - 推定: `python3 -m pytest src/test_estimation.py -q`  
  - コンタクト/プランナ: `python3 -m pytest src/test_contact_state_machine.py src/test_com_planner.py -q`
- 長時間/診断（任意, デフォルトスキップ）:  
  - ロバスト性: `RUN_ROBUSTNESS=1 python3 -m pytest src/test_phase46_robustness.py`  
  - 診断IK/歩行: `RUN_DIAGNOSTICS=1 python3 -m pytest src/diagnostics/test_ik_walking.py src/diagnostics/test_walking_detailed.py`  
  - 注意: PyBullet が長時間診断で不安定になる場合あり。

## 4. 重要な共有設定
- 基準高さ・立位姿勢は `robot_constants.py` の `BASE_HEIGHT=0.679` と `STANDING_CONFIG` を使用（ハードコード禁止）。
- テスト共通ヘルパ: `test_helpers.py`（URDFパス/立位姿勢）。

## 5. モードの位置づけ
- `--mode walking`: 位置制御歩行（ZMPベースCoM + 全身IK + POSITION_CONTROL）。現行の安定歩行パス。
- `--mode wbc`: MPC+WBC 立位/姿勢用。歩行には未使用。
- `--mode standing` / `standing-mpc`: PD / MPC 立位。
- `wbc_walking_controller.py` は参考・再設計中（既定モードでは未使用）。

## 6. ログ/成果物
- 画像・ログは `logs/` に保存（PNG は通常コミット対象外推奨）。
- 過去フェーズ資料は `obsolete_docs/` に移動済み。現行の手順/規約は `AGENTS.md` を参照。

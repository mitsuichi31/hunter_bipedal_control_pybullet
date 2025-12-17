# Hunter 2足歩行ロボット シミュレーション

PyBullet を用いた Hunter 二足歩行ロボットのシミュレーション環境。立位・バランス・歩行を対象に、PD/MPC/WBC/位置制御歩行の各モードを備えています。

## 現在の状態
- 立位系: `standing`（PD）、`standing-mpc`（MPC+ZMP）とも安定動作。
- WBC: `wbc` 立位は Phase 2 安定版。歩行系は再設計中（参考実装あり）。
- 位置制御歩行: `walking` は位置制御+ZMP CoM+全身IKで安定（回帰テストあり）。
- 基準姿勢/高さ: `BASE_HEIGHT=0.679`、`STANDING_CONFIG` を全コントローラ/テストで共通化。

## 主要ファイル
- 制御: `position_control_walking.py`（位置制御歩行）, `mpc_wbc_controller.py`（MPC+WBC統合）, `wbc_walking_controller.py`（WBC歩行・再設計中）
- 計画/推定: `com_planner.py` / `com_planner_simple.py`, `estimation/observer.py`, `estimation/state_filter.py`, `estimation/contact_estimator.py`
- 補助: `robot_constants.py`（基準高さ・立位姿勢）, `test_helpers.py`（URDF/立位共通ヘルパ）
- テスト: `test_position_control_walking_regression.py`（歩行回帰）, `test_wbc_forward_velocity.py`, `test_contact_state_machine.py`, `test_estimation.py` ほか各機能ごとの `test_*`
- ドキュメント: 現行ガイドは `AGENTS.md`。過去フェーズ資料は `obsolete_docs/` に移動。

## 実行例
- 立位（PD, GUI無効）  
  `python3 src/main_simulation.py --mode standing --duration 10 --no-gui`
- 位置制御歩行（GUI無効）  
  `python3 src/main_simulation.py --mode walking --duration 12 --no-gui`

Docker 推奨: `cd docker && docker-compose up -d` 後、コンテナ内で上記コマンドを実行。

## テスト
- フルハーネス（コンテナ内）: `scripts/test_all_modes.sh`  
  - 位置制御歩行の回帰 (`test_position_control_walking_regression.py`) を含む
- 単体: `python3 -m pytest src/test_estimation.py`（推定）、`python3 -m pytest src/test_contact_state_machine.py`
- 長時間/診断系は環境変数で有効化:  
  - ロバストネス: `RUN_ROBUSTNESS=1 python3 -m pytest src/test_phase46_robustness.py`  
  - 診断IK/歩行: `RUN_DIAGNOSTICS=1 python3 -m pytest src/diagnostics/test_ik_walking.py src/diagnostics/test_walking_detailed.py`
  - 既知: PyBullet が長時間/診断テストで不安定になる場合あり（セグフォールト）。

## 開発メモ
- 共有定数: 基準高さ/立位姿勢は `robot_constants.py` を参照し、ハードコードを避ける。
- テスト共通化: URDF/立位姿勢は `test_helpers.py` を利用。
- ログ/PNG は `logs/` に出力（CI向けにはコミット対象外を推奨）。
- 過去のフェーズ資料・古いプランは `obsolete_docs/` へ移動済み。

## 貢献
コーディング規約・テスト手順は `AGENTS.md` を参照。Pull Request では実行コマンドと結果を記載してください。

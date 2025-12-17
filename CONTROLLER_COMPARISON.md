# コントローラ比較（概要）

本リポジトリにある主要な歩行・立位コントローラの特徴と現状をまとめます。

## 位置制御歩行（既定・安定）
- ファイル: `position_control_walking.py`
- 構成: `gait_generator` → ZMPベース CoM プランナー（`com_planner_simple`）→ 全身IK（`full_body_ik`）→ PyBullet POSITION_CONTROL
- 状態: 安定動作。`--mode walking` で使用。回帰テスト `test_position_control_walking_regression.py` あり。

## MPC+WBC 統合（立位・姿勢）
- ファイル: `mpc_wbc_controller.py`
- 構成: LIPMベース MPC で CoM/ZMP 目標生成 → WBC (`wbc_controller.py`) で接地点力/トルク算出。ポスチャには共有 `STANDING_CONFIG` を使用。
- 状態: 立位/前進スタンス検証 (`test_wbc_forward_velocity.py`) で利用。歩行モードには未使用。

## WBC 歩行（参考・再設計中）
- ファイル: `wbc_walking_controller.py`
- 目的: 接触状態機械 + タスク階層 + QP による歩行（WBC）。現状は参考実装・再設計中。`WBC_ARCHITECTURAL_REDESIGN.md` 参照。
- 状態: 既定の歩行パスでは未使用。

## 比較のポイント
- 安定性/用途:  
  - 位置制御歩行: 最も安定。回帰テストで守られており GUI/ヘッドレス両対応。  
  - MPC+WBC: 立位/姿勢で使用。歩行には未統合。  
  - WBC歩行: 再設計中の参考コード。
- 共有定数: 基準高さ `BASE_HEIGHT=0.679`、立位姿勢 `STANDING_CONFIG` を全コントローラで共通化（`robot_constants.py`）。
- テストカバレッジ:  
  - 歩行回帰: `test_position_control_walking_regression.py`  
  - WBCスタンス: `test_wbc_forward_velocity.py`  
  - コンタクト/プランナ: `test_contact_state_machine.py`, `test_com_planner.py`
- 既知の課題:  
  - 長時間/診断系（RUN_ROBUSTNESS/RUN_DIAGNOSTICS）は PyBullet が不安定になる場合あり。デフォルトではスキップ。  
  - WBC 歩行はアーキテクチャ調整が必要（姿勢/タスクの一貫性、ハイブリッド制御の扱い）。

## モードの使い分け
- `--mode walking`: 位置制御歩行（安定版）
- `--mode wbc`: MPC+WBC 立位（統合コントローラ）
- `--mode standing` / `--mode standing-mpc`: PD / MPC 立位
- WBC 歩行（`wbc_walking_controller.py`）は既定モードでは呼ばれません。

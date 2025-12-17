# 制御システム概要（Hunter 二足歩行）

本リポジトリに実装されている主要な制御系と現状の使い分けをまとめます。

## 現行の歩行パス
- **位置制御歩行（安定・既定）**  
  - ファイル: `position_control_walking.py`  
  - 構成: `gait_generator` → ZMPベース CoM プランナー (`com_planner_simple`) → 全身IK (`full_body_ik`) → PyBullet POSITION_CONTROL。  
  - 状態: 安定動作。回帰テスト `test_position_control_walking_regression.py` あり。`--mode walking` で使用。
- **MPC+WBC 統合（立位/姿勢用）**  
  - ファイル: `mpc_wbc_controller.py`  
  - 構成: LIPM ベース MPC (`mpc_controller.py`) で CoM/ZMP 目標生成 → WBC (`wbc_controller.py`) で接地点力/トルク算出。  
  - 状態: 立位・前進速度スタンス検証 (`test_wbc_forward_velocity.py`) で使用。歩行経路には未使用。

## WBC 歩行（参考・再設計中）
- **ファイル:** `wbc_walking_controller.py`  
  - 目的: WBC を用いた歩行アーキテクチャ（接触 FSM + タスク階層 + QP）。  
  - 状態: 参考実装・再設計中。`WBC_ARCHITECTURAL_REDESIGN.md` を参照。既定の歩行パスでは未使用。

## 共有定数とヘルパ
- **`robot_constants.py`**: 基準高さ `BASE_HEIGHT=0.679`、立位姿勢 `STANDING_CONFIG` を全制御系・テストで共通化。
- **`test_helpers.py`**: テスト向け URDF パス・立位設定ヘルパ。

## テストと運用
- 位置制御歩行回帰: `python3 -m pytest src/test_position_control_walking_regression.py -q`
- WBC 前進スタンス検証: `python3 -m pytest src/test_wbc_forward_velocity.py -q`
- フルハーネス（コンテナ内）: `scripts/test_all_modes.sh`（位置制御歩行回帰を含む）
- 長時間/診断系は環境変数で任意実行:  
  - ロバスト性: `RUN_ROBUSTNESS=1 python3 -m pytest src/test_phase46_robustness.py`  
  - 診断 IK/歩行: `RUN_DIAGNOSTICS=1 python3 -m pytest src/diagnostics/test_ik_walking.py src/diagnostics/test_walking_detailed.py`  
  - 既知: PyBullet が長時間/診断で不安定になる場合あり（セグフォールト）。デフォルトではスキップ。

## モードの使い分け
- `--mode standing` / `--mode standing-mpc`: 立位（PD または MPC）。  
- `--mode wbc`: WBC 立位（MPC+WBC 統合）。  
- `--mode walking`: 位置制御歩行（安定版）。  
- WBC 歩行（再設計中）は既定モードからは呼ばれません。

## 今後の改善メモ
- WBC 歩行アーキテクチャの再設計と統合検証。  
- 位置制御歩行のさらなるユーティリティ分離（共通接触判定・診断ロガ）。  
- テストの共通化・警告削減を継続。

# 歩行アプローチ比較（現状まとめ）

**現行の安定パス**: 位置制御歩行（`position_control_walking.py`）  
**参考/再設計中**: WBC歩行（`wbc_walking_controller.py`）  
**立位統合**: MPC+WBC（`mpc_wbc_controller.py`）は立位/姿勢用、歩行には未使用。

## アプローチ別概要
- **位置制御 + ZMP CoM + 全身IK（安定・既定）**  
  - 構成: `gait_generator` → `com_planner_simple` → `full_body_ik` → POSITION_CONTROL  
  - 状態: 安定。`--mode walking` で使用。回帰テスト `test_position_control_walking_regression.py` あり。
- **MPC+WBC（立位/姿勢）**  
  - 構成: LIPM MPC (`mpc_controller.py`) → WBC (`wbc_controller.py`) → ハイブリッド/トルク出力。  
  - 状態: 立位系で使用 (`test_wbc_forward_velocity.py`); 歩行には未統合。
- **WBC歩行（再設計中・参考）**  
  - 構成: 接触FSM + タスク階層 + WBC QP。  
  - 状態: 参考実装、再設計計画は `WBC_ARCHITECTURAL_REDESIGN.md` 参照。既定の歩行パスからは外れている。

## 使い分け
- 本番/検証: 位置制御歩行を既定で使用。  
- WBC立位: MPC+WBC パスを立位/姿勢で利用。  
- WBC歩行: 現時点で研究・再設計中。安定版ではない。

## 関連テスト
- 歩行回帰: `python3 -m pytest src/test_position_control_walking_regression.py -q`
- WBC前進スタンス: `python3 -m pytest src/test_wbc_forward_velocity.py -q`
- 参考/長時間: RUN_ROBUSTNESS, RUN_DIAGNOSTICS 環境変数で任意実行（PyBullet 不安定性に注意）。

# WBC 歩行診断ノート（現状整理）

**ステータス**: WBC 歩行は参考/再設計中。現在の歩行は位置制御パスを使用。立位/MPC+WBC パスは安定。

## 現行の使い分け
- 歩行: `position_control_walking.py`（ZMP CoM + 全身IK + POSITION_CONTROL）
- 立位/姿勢: `mpc_wbc_controller.py`（MPC + WBC、`STANDING_CONFIG` 共有）
- WBC歩行: `wbc_walking_controller.py` は研究用（既定モードでは未使用）。

## 留意点
- 過去の詳細な診断・フェーズ別ログは `obsolete_docs/` に移動済み。
- 長時間/診断テストは RUN_DIAGNOSTICS/RUN_ROBUSTNESS 環境変数で任意実行（PyBullet 不安定性に注意）。
- 歩行回帰テスト (`test_position_control_walking_regression.py`) を基準に、WBC歩行統合の将来評価を行う。

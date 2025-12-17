# WBC 歩行アーキテクチャ再設計メモ（現状）

**ステータス**: 参考/再設計中（既定歩行パス外）。立位・歩行には現行の位置制御歩行/MPC+WBC 立位を使用。

## 現状整理
- MPC+WBC 立位（`mpc_wbc_controller.py`）は安定。歩行への統合は未実施。
- WBC 歩行（`wbc_walking_controller.py`）は接触FSM+タスク階層の参考実装。安定版ではない。
- 位置制御歩行（`position_control_walking.py`）が実運用の歩行パス。

## 再設計の方向性（検討用）
- 立位で確認済みのタスク最小構成（姿勢+CoM）と共有定数 (`STANDING_CONFIG`, `BASE_HEIGHT`) を流用。
- 接触遷移・足アンカーの扱いと IK/CoM 計画の整合性を優先。  
- トルク/ハイブリッド制御の再導入は実験的扱いとし、まずは位置制御/力アンカーとのハイブリッド案を段階評価。

## 参考テスト/資料
- `test_wbc_forward_velocity.py`（立位ハーネス）で立位/姿勢パラメータを検証。
- 位置制御歩行の回帰 (`test_position_control_walking_regression.py`) を基準に、歩行での WBC 統合を評価予定。
- 過去の詳細ログ/分析は `obsolete_docs/` のフェーズ資料に保管。

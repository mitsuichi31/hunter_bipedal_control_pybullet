# クイックスタートガイド

Hunter二足歩行ロボットシミュレーションを最速で始めるためのガイドです。

## ⚠️ 重要：現在の機能状況

| モード | 状態 | 推奨度 |
|--------|------|--------|
| `standing` | ✅ 完璧 | ⭐⭐⭐⭐⭐ |
| `standing-mpc` | ✅ 完璧 | ⭐⭐⭐⭐⭐ |
| `wbc` | ✅ Phase 2完了 | ⭐⭐⭐⭐ |
| `walking` | 🔍 Phase 3調査完了・再設計中 | ⭐⭐ |

**推奨**: `standing`, `standing-mpc`, `wbc` モードが動作します！
**Phase 3**: WBC-ハイブリッド制御の調査完了。アーキテクチャ再設計計画作成済み。

## 5分で始める

### ステップ1: Docker環境起動

```bash
# Docker コンテナを起動
cd docker
docker-compose up -d

# 動作確認
docker ps  # hunter-simulationが表示されればOK
```

### ステップ2: 立位テスト（動作確認済み ✅）

```bash
# GUIなしで立位テスト（最速）
docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode standing --duration 5 --no-gui
```

**期待される出力:**
```
✓ Robot is upright!
  Roll angle: 0.2° (within limits)
  Pitch angle: 0.1° (within limits)
```

この出力が表示されれば成功です！

### ステップ3: 全モードテスト

```bash
# すべてのモードを一度にテスト
docker exec hunter-simulation bash /workspace/hunter/scripts/test_all_modes.sh
```

**期待される出力:**
```
[1/4] Testing STANDING mode...
✓ Robot is upright! Roll: 0.2°, Pitch: 0.1°    ← 成功！

[2/4] Testing STANDING-MPC mode...
✓ Robot is upright! Roll: 0.2°, Pitch: 0.1°    ← 成功！

[3/4] Testing WBC mode...
✓ Robot is upright! Roll: 0.0°, Pitch: 0.0°    ← Phase 2完了！

[4/4] Testing WALKING mode...
Note: Currently maintains standing position      ← Phase 3で実装予定
```

## 動作するモード ✅

### Standing（立位保持）- 推奨 ⭐⭐⭐⭐⭐

最もシンプルで安定したモードです：

```bash
# GUIあり（視覚的確認）
docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode standing --duration 10

# GUIなし（高速）
docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode standing --duration 10 --no-gui
```

- **制御方式**: PD制御
- **安定性**: ✅ 完璧（Roll=0.2°, Pitch=0.1°）
- **用途**: 基本的な姿勢制御の学習、パラメータ調整
- **無期限安定**: ロボットは永遠に立ち続けます

### Standing-MPC（MPC立位制御）- 推奨 ⭐⭐⭐⭐⭐

Model Predictive Controlによるバランス制御：

```bash
docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode standing-mpc --duration 10 --no-gui
```

- **制御方式**: MPC + ZMP + PD制御
- **安定性**: ✅ 完璧（Roll=0.2°, Pitch=0.1°）
- **用途**: 高度なバランス制御の学習
- **特徴**: 最小限の補正で立位を維持

## Phase 2完了モード ✅

### WBC（Whole-Body Control）- 立位制御完成

```bash
# Phase 2完了 - WBC立位制御テスト
docker exec hunter-simulation python3 /workspace/hunter/src/test_wbc_standing.py
```

**または標準シミュレーション:**
```bash
docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode wbc --duration 10 --no-gui
```

- **状態**: ✅ Phase 2完了（Roll=0.00°, Pitch=0.03°）
- **成果**:
  - WBCパラメータチューニング完了
  - 地面反力最適化誤差0.1%
  - QP最適化100%実行可能
  - 10秒立位テスト完璧に安定
- **新機能**:
  - 正確なCoM/ZMP計算（Phase 1）
  - 重力補償（Phase 1）
  - 逆動力学実装（Phase 2）
- **用途**: 高度なWBC制御、バランス制御の研究

## 開発中のモード ⚠️

### Walking（歩行）- 研究段階

```bash
docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode walking --duration 5 --no-gui
```

- **現状**: 立位姿勢を維持するのみ
- **制限**: 実際の歩行動作は未実装
- **理由**:
  - 二足歩行には逆動力学モデルが必要
  - 接触力最適化が必要
  - 現在のIKベースのアプローチでは不十分
- **詳細**: [WALKING_MODE_INVESTIGATION.md](WALKING_MODE_INVESTIGATION.md)参照

## GUI表示について（オプション）

GUIを使いたい場合のみ設定が必要です。

### Linux / WSL2

```bash
# X11アクセスを許可
xhost +local:docker

# WSL2の場合、DISPLAYを設定
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

VcXsrvなどのX11サーバーをWindows側で起動してください。

### macOS

```bash
# XQuartzをインストール（初回のみ）
brew install --cask xquartz

# XQuartzを起動
open -a XQuartz

# DISPLAYを設定
export DISPLAY=host.docker.internal:0
xhost + 127.0.0.1
```

### GUIで実行

```bash
docker exec -it hunter-simulation bash
cd /workspace/hunter/src
python3 main_simulation.py --mode standing --duration 10
```

## 重要なパラメータ

### ⚠️ 最も重要：ベース高さ

```python
base_height = 0.679  # m
```

**これは絶対に変更しないでください！** 他の値では転倒します。

この値は詳細な分析によって発見された正確な値です。

### PD制御ゲイン

```python
# 標準関節
default_kp = 200.0
default_kd = 20.0

# 重要な関節（膝、股関節ピッチ）
critical_kp = 300.0
critical_kd = 30.0
```

### 立位姿勢設定（変更しないでください）

```python
standing_config = {
    # まっすぐな脚、わずかに外側のスタンス
    'leg_l1_joint': -0.1,  # 左股関節ロール
    'leg_l2_joint': 0.0,
    'leg_l3_joint': 0.0,
    'leg_l4_joint': 0.0,
    'leg_l5_joint': 0.0,
    'leg_r1_joint': 0.1,   # 右股関節ロール
    'leg_r2_joint': 0.0,
    'leg_r3_joint': 0.0,
    'leg_r4_joint': 0.0,
    'leg_r5_joint': 0.0,
}
```

この設定はRoll=0.2°の完璧な安定性を達成します。

## よくある質問

### Q: WBCモードは使えますか？

A: ✅ **はい！** Phase 2が完了し、WBC立位制御が動作します（Roll=0.00°, Pitch=0.03°）。`test_wbc_standing.py`で検証テストを実行できます。

### Q: 歩行はできますか？

A: 現時点では歩行は実装されていません（Phase 3で実装予定）。歩行にはWBC歩行アーキテクチャの実装が必要で、これは次のマイルストーンです。詳細は[STABILITY_IMPROVEMENT_PLAN.md](STABILITY_IMPROVEMENT_PLAN.md)と[WALKING_MODE_INVESTIGATION.md](WALKING_MODE_INVESTIGATION.md)を参照してください。

### Q: どのモードを使えばいいですか？

A: **`standing`, `standing-mpc`, `wbc`モード**が全て動作します。`standing`と`standing-mpc`は最も安定しています。`wbc`はより高度な制御を提供します。

### Q: ロボットが倒れます

A: 以下を確認してください：
1. ✅ `base_height = 0.679` になっているか
2. ✅ `standing`, `standing-mpc`, `wbc`のいずれかのモードを使用しているか
3. ✅ まっすぐな脚の設定を使用しているか

`walking`モードで転倒するのは既知の問題です（Phase 3で修正予定）。

### Q: パラメータを変更したい

A: ⚠️ **注意**: デフォルトのパラメータは最適化されています。
- `base_height = 0.679` は変更しないでください
- `standing_config`の関節角度は変更しないでください
- PD制御ゲインは慎重に調整してください

実験する場合は、まずコピーを作成してください。

## トラブルシューティング

### ロボットが倒れる

**standingモードで倒れる場合：**
1. ✅ `base_height = 0.679` を確認
2. ✅ コードが最新版か確認
3. ✅ まっすぐな脚の設定を使用

**wbcモードで倒れる場合：**
- Phase 2が完了しているか確認（2025年11月24日以降のコード）
- `test_wbc_standing.py`で検証テストを実行

**walkingモードで倒れる場合：**
- これは既知の問題です（Phase 3で修正予定）。`standing`, `standing-mpc`, `wbc`を使用してください。

### Docker環境のトラブル

```bash
# コンテナの再起動
docker-compose restart

# ログの確認
docker logs hunter-simulation

# コンテナ内に入る
docker exec -it hunter-simulation bash
```

### GUIが表示されない

**解決方法1**: GUIなしで実行（推奨）
```bash
docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode standing --duration 10 --no-gui
```

**解決方法2**: X11設定を確認
- WSL2: VcXsrvが起動しているか確認
- Linux: `xhost +local:docker` を実行
- macOS: XQuartzが起動しているか確認

## コマンド早見表

```bash
# ===== 動作確認済み（推奨）=====
# 立位テスト（GUIなし）
docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode standing --duration 5 --no-gui

# MPC立位テスト（GUIなし）
docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode standing-mpc --duration 5 --no-gui

# 全モードテスト
docker exec hunter-simulation bash /workspace/hunter/scripts/test_all_modes.sh

# ===== 開発中（転倒します）=====
# WBCテスト（転倒することに注意）
docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode wbc --duration 5 --no-gui

# 歩行テスト（立位のみ）
docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode walking --duration 5 --no-gui

# ===== コンテナ管理 =====
# コンテナ起動
cd docker && docker-compose up -d

# コンテナ停止
cd docker && docker-compose down

# コンテナ内に入る
docker exec -it hunter-simulation bash
```

## 次のステップ

### 学習

1. **README.md** - 完全なドキュメント
2. **STABILITY_FIX.md** - なぜRoll=0.2°が達成できるのか
3. **MPC_WALKING_FIX.md** - MPC制御の詳細

### 実験

```bash
# 診断ツール：安定姿勢解析
docker exec hunter-simulation python3 /workspace/hunter/src/diagnostics/find_stable_pose.py

# 異なる期間でテスト
docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode standing --duration 30 --no-gui

# GUIで視覚的に確認
docker exec -it hunter-simulation bash
python3 /workspace/hunter/src/main_simulation.py --mode standing --duration 10
```

### 開発

1. ソースコードを読む（特に`balance_controller.py`）
2. パラメータを慎重に調整
3. WBCモードのパラメータチューニングに挑戦

## 期待値の設定

### 動作するもの ✅
- 立位保持（standing）- Roll=0.2°
- MPC立位制御（standing-mpc）- Roll=0.2°
- WBC立位制御（wbc）- Roll=0.00°, Pitch=0.03° (Phase 2完了)
- 完璧な安定性

### 動作しないもの ❌
- 歩行（walkingモード - Phase 3で実装予定）

### 学べるもの 📚
- 二足歩行ロボットの基本
- PD制御、MPC制御、WBC制御
- 安定性分析（CoM、ZMP、安定性マージン）
- 重力補償と逆動力学
- QP最適化によるバランス制御
- PyBulletシミュレーション

## 困ったときは

1. **[README.md](README.md)** を確認
2. **[STABILITY_FIX.md](STABILITY_FIX.md)** で技術詳細を確認
3. GitHubのIssueで質問
4. `docker logs hunter-simulation` でログ確認

## 成功の確認

以下が表示されれば成功です：

```
✓ Robot is upright!
  Roll angle: 0.2° (within limits)
  Pitch angle: 0.1° (within limits)
```

これはRoll=0.2°、Pitch=0.1°という、ほぼ完璧な直立姿勢です！

Happy Simulating! 🤖

---

**推奨コマンド**: `docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode standing --duration 10 --no-gui`

**動作モード**: `standing`, `standing-mpc`, `wbc` ✅

**詳細ドキュメント**: [README.md](README.md) | [STABILITY_IMPROVEMENT_PLAN.md](STABILITY_IMPROVEMENT_PLAN.md)

**現在の状態**: 立位制御 ✅ 完成 | WBC ✅ Phase 2完了 | 歩行 ⚠️ Phase 3予定

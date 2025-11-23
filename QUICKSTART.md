# クイックスタートガイド

Hunter二足歩行ロボットシミュレーションを最速で始めるためのガイドです。

## ⚠️ 重要：現在の機能状況

| モード | 状態 | 推奨度 |
|--------|------|--------|
| `standing` | ✅ 完璧 | ⭐⭐⭐⭐⭐ |
| `standing-mpc` | ✅ 完璧 | ⭐⭐⭐⭐⭐ |
| `wbc` | ⚠️ 転倒する | ⭐⭐ |
| `walking` | ⚠️ 立位のみ | ⭐⭐ |

**推奨**: まず`standing`または`standing-mpc`モードから始めてください！

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
✗ FAILED: Robot has fallen                      ← 既知の問題

[4/4] Testing WALKING mode...
Note: Currently maintains standing position      ← 開発中
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

## 開発中のモード ⚠️

### WBC（Whole-Body Control）- パラメータチューニング必要

```bash
docker exec hunter-simulation python3 /workspace/hunter/src/main_simulation.py --mode wbc --duration 10 --no-gui
```

- **現状**: ✗ ロボットが転倒（Roll=108.7°）
- **原因**: QP最適化のパラメータが最適化されていない
- **課題**:
  - 制御ゲインの調整が必要
  - 摩擦円錐制約のチューニングが必要
  - 接地力最適化の重み付け調整が必要
- **用途**: WBC研究、上級者向け

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

### Q: なぜWBCモードが転倒するのですか？

A: WBCコードは統合されていますが、QP最適化のパラメータがまだ最適化されていません。これは既知の問題で、パラメータチューニングが必要です。`standing`または`standing-mpc`モードは完璧に動作します。

### Q: 歩行はできますか？

A: 現時点では歩行は実装されていません。歩行には逆動力学モデル（Pinocchioなど）の統合が必要で、これは将来の開発課題です。詳細は[WALKING_MODE_INVESTIGATION.md](WALKING_MODE_INVESTIGATION.md)を参照してください。

### Q: どのモードを使えばいいですか？

A: **`standing`または`standing-mpc`モード**を使ってください。これらは完璧に動作し、安定しています。

### Q: ロボットが倒れます

A: 以下を確認してください：
1. ✅ `base_height = 0.679` になっているか
2. ✅ `standing`または`standing-mpc`モードを使用しているか
3. ✅ まっすぐな脚の設定を使用しているか

`wbc`や`walking`モードで転倒するのは既知の問題です。

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
- これは既知の問題です。`standing`または`standing-mpc`を使用してください。

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
- 立位保持（standing）
- MPC立位制御（standing-mpc）
- 完璧な安定性（Roll=0.2°）

### 動作しないもの ❌
- 歩行（walkingモード - 研究段階）
- WBC（wbcモード - チューニング必要）

### 学べるもの 📚
- 二足歩行ロボットの基本
- PD制御とMPC制御
- 安定性分析
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

**最も安定**: `standing` と `standing-mpc` モード ✅

**詳細ドキュメント**: [README.md](README.md)

**現在の状態**: 立位制御 ✅ 完成 | WBC ⚠️ 調整中 | 歩行 ⚠️ 研究段階

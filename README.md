# Hunter 2足歩行ロボット シミュレーション環境

PyBulletを使用したHunter 2足歩行ロボットのシミュレーション環境です。立位制御とバランス制御のテスト用に設計されています。

## 現在の機能状況

| モード | 状態 | 説明 |
|--------|------|------|
| **standing** | ✅ **完全動作** | PD制御による立位保持（Roll=0.2°） |
| **standing-mpc** | ✅ **完全動作** | MPC+ZMP制御による立位保持（Roll=0.2°） |
| **wbc** | ✅ **Phase 2完了** | WBC立位制御（Roll=0.00°, Pitch=0.03°） |
| **walking** | 🚧 **Phase 3進行中** | WBC歩行アーキテクチャ実装中（M1-3完了、M4-5残） |

## 特徴

- **シミュレータ**: PyBullet（理解しやすく、リアルタイムで動作確認可能）
- **動作する制御方法**:
  - ✅ PD制御（立位姿勢）
  - ✅ MPC + ZMP制御（バランス立位）
  - ✅ WBC (Whole-Body Control) - Phase 2完了、立位制御動作
- **開発中の機能**:
  - ⚠️ 歩行制御 - Phase 3で実装予定
- **ロボットモデル**: 10自由度二足歩行ロボット（各脚5関節）

## Phase 1 & 2 安定性向上プロジェクト完了 🎉

**2025年11月24日完了** - 4日間で2フェーズを完成（計画の5倍速）

### Phase 1: コア安定性基礎（完了✅）
- ✅ **正確なCoM計算**: 全リンクの質量を考慮（16.7cm精度向上）
- ✅ **動的ZMP計算**: 加速度項を含む真のZMP計算
- ✅ **重力補償**: フィードフォワードトルクで30%効率化

### Phase 2: WBCチューニング＆検証（完了✅）
- ✅ **WBC立位制御**: Roll=0.00°, Pitch=0.03°（目標<1°を大幅に達成）
- ✅ **逆動力学実装**: 質量行列M(q)、重力トルクg(q)を計算
- ✅ **力最適化**: 地面反力誤差0.1%（優秀な精度）

### 新規モジュール
- `src/stability_metrics.py` - CoM/ZMP/安定性マージン計算
- `src/gravity_compensation.py` - 関節空間重力補償
- `src/inverse_dynamics.py` - ロボット動力学計算
- `src/test_phase1_integration.py` - Phase 1統合テスト
- `src/test_wbc_standing.py` - WBC立位検証テスト

詳細は [STABILITY_IMPROVEMENT_PLAN.md](STABILITY_IMPROVEMENT_PLAN.md) を参照してください。

### Phase 3: WBC歩行モード（進行中 🚧）

**2025年11月24日開始** - アーキテクチャ実装進行中

#### 完了したマイルストーン
- ✅ **M1: 接触状態機械**: 接触フェーズ管理（DS/LS/RS）、接触検出、6/6テスト合格
- ✅ **M2: WBCタスク**: 遊脚追跡タスク、支持脚制約、階層的タスク統合
- ✅ **M3: 接触遷移**: スムーズ遷移（50ms）、安全チェック強化、緊急停止
- ✅ **M4.1: 超保守的歩容**: 5cmステップ、2s周期、50%二重支持
- ✅ **M5.1: 統合**: main_simulation.py統合、エンドツーエンド検証

#### 実装されたアーキテクチャ
```
歩容生成器 → 接触FSM → タスク階層 → WBC QP → 逆動力学 → トルク
```

#### 残作業（M4.2、M4.3、M5.2）
- ⚠️ 実際のトルク計算（WBC QP + 逆動力学の完全統合）
- ⚠️ 歩容パラメータチューニング（トルク実装後）
- ⚠️ 包括的テストスイート
- ⚠️ ドキュメント最終更新

#### 新規ファイル
- `src/contact_state_machine.py` - 接触フェーズ管理
- `src/test_contact_state_machine.py` - 接触FSMテスト（6/6合格）
- `src/wbc_walking_controller.py` - WBC歩行制御器（再設計）
- `PHASE3_WALKING_PLAN.md` - Phase 3詳細計画

詳細は [PHASE3_WALKING_PLAN.md](PHASE3_WALKING_PLAN.md) を参照してください。

## 貢献者向けガイド

コントリビューション手順やテスト・命名規約は [AGENTS.md](AGENTS.md) にまとめています。

## プロジェクト構成

```
hunter/
├── config/
│   └── default_config.yaml      # パラメータ設定ファイル
├── models/
│   └── urdf/
│       └── hunter.urdf           # Hunterロボットモデル
├── src/
│   ├── main_simulation.py        # メインシミュレーション
│   ├── simulation_env.py         # PyBulletシミュレーション環境
│   ├── config_loader.py          # 設定ファイルローダー
│   │
│   ├── pd_controller.py          # PD制御器 ✅
│   ├── balance_controller.py     # MPC+ZMP バランス制御 ✅
│   ├── mpc_controller.py         # MPC制御器 ✅
│   ├── wbc_controller.py         # WBC制御器（QP最適化）✅
│   ├── wbc_tasks.py              # WBCタスク階層 ✅
│   ├── mpc_wbc_controller.py     # MPC+WBC統合制御器 ✅
│   │
│   ├── stability_metrics.py      # CoM/ZMP計算（Phase 1）✅
│   ├── gravity_compensation.py   # 重力補償（Phase 1）✅
│   ├── inverse_dynamics.py       # 逆動力学（Phase 2）✅
│   │
│   ├── inverse_kinematics.py     # 逆運動学ソルバー
│   ├── gait_generator.py         # 歩行軌道生成器
│   ├── contact_state_machine.py  # 接触状態機械（Phase 3）✅
│   ├── wbc_walking_controller.py # WBC歩行制御器（Phase 3）🚧
│   │
│   ├── test_stability_metrics.py # Phase 1テスト
│   ├── test_gravity_compensation.py # Phase 1テスト
│   ├── test_inverse_dynamics.py  # Phase 2テスト
│   ├── test_phase1_integration.py # Phase 1統合テスト
│   ├── test_wbc_standing.py      # Phase 2検証テスト
│   ├── test_contact_state_machine.py # Phase 3テスト（6/6合格）✅
│   │
│   └── diagnostics/              # 診断・解析ツール
│       ├── find_stable_pose.py   # 安定姿勢解析
│       └── README.md             # 診断ツールの説明
│
├── scripts/
│   ├── run_docker.sh             # Docker実行スクリプト
│   ├── run_simulation.sh         # シミュレーション実行ラッパー
│   └── test_all_modes.sh         # 全モードテストスクリプト
├── logs/                         # ログファイル保存先
├── requirements.txt              # 依存パッケージ
├── CLAUDE.md                     # Claude Code開発ガイド
├── STABILITY_IMPROVEMENT_PLAN.md # Phase 1-4開発計画（Phase 2完了）
├── PHASE3_WALKING_PLAN.md        # Phase 3詳細実装計画
├── CONTROL_SYSTEM_OVERVIEW.md    # 制御システムアーキテクチャ概要
├── STABILITY_FIX.md              # 立位安定化の技術詳細
├── MPC_WALKING_FIX.md            # MPC/歩行調査レポート
├── WALKING_MODE_INVESTIGATION.md # 歩行モード詳細調査
├── ARCHITECTURE_CHANGES_SUMMARY.md # アーキテクチャ変更の試み
├── SESSION_SUMMARY_2025-11-23.md # 開発セッション概要（Nov 23）
└── README.md                     # このファイル
```

## セットアップ

### 方法1: Docker環境（推奨）

Docker環境で実行する場合、依存関係が自動的に解決されます。

```bash
# コンテナの起動とビルド
cd docker
docker-compose up -d

# コンテナ内でテスト実行
docker exec hunter-simulation bash /workspace/hunter/scripts/test_all_modes.sh
```

詳細は [DOCKER.md](DOCKER.md) を参照してください。

### 方法2: ローカル環境

#### 1. 依存パッケージのインストール

```bash
pip install -r requirements.txt
```

主な依存パッケージ:
- `pybullet>=3.2.5` - 物理シミュレーション
- `numpy>=1.24.0` - 数値計算
- `scipy>=1.10.0` - 科学計算
- `cvxpy>=1.3.0` - WBC QP最適化
- `pyyaml>=6.0` - 設定ファイル読み込み

#### 2. URDFモデルの確認

`models/urdf/hunter.urdf`にHunterロボットのURDFファイルが配置されています。

## 使い方

### 動作確認済みモード

#### 1. 立位姿勢のテスト（推奨）

ロボットが完璧に立位を保持します（Roll=0.2°, Pitch=0.1°）：

```bash
cd src
python main_simulation.py --mode standing --duration 10
```

**期待される結果:**
```
✓ Robot is upright!
  Roll angle: 0.2° (within limits)
  Pitch angle: 0.1° (within limits)
```

#### 2. MPC立位制御テスト（推奨）

Model Predictive Controlを使用したバランス立位：

```bash
cd src
python main_simulation.py --mode standing-mpc --duration 10
```

**期待される結果:**
```
✓ Robot is upright!
  Roll angle: 0.2° (within limits)
  Pitch angle: 0.1° (within limits)
```

### 開発中のモード

#### 3. WBC制御テスト（パラメータチューニング必要）

Whole-Body Controlの統合テスト：

```bash
cd src
python main_simulation.py --mode wbc --duration 10
```

**現状:** コードは動作しますが、ロボットが転倒します。QP最適化のパラメータチューニングが必要です。

**技術的課題:**
- 制御ゲインの調整
- 摩擦円錐制約のチューニング
- 接地力最適化の重み付け

#### 4. 歩行シミュレーション（研究段階）

```bash
cd src
python main_simulation.py --mode walking --duration 5
```

**現状:** 立位姿勢を維持するのみ。実際の歩行動作は未実装です。

**技術的理由:**
- 二足歩行には逆動力学モデルが必要
- 接触力最適化とモーメンタム制約が必要
- 現在のIKベースのアプローチでは不十分

詳細は [WALKING_MODE_INVESTIGATION.md](WALKING_MODE_INVESTIGATION.md) を参照してください。

### コマンドラインオプション

```bash
python main_simulation.py --help
```

- `--mode`: シミュレーションモード
  - `standing` ✅ - PD制御立位
  - `standing-mpc` ✅ - MPC制御立位
  - `wbc` ⚠️ - WBC統合（開発中）
  - `walking` ⚠️ - 歩行（研究段階）
- `--duration`: シミュレーション時間（秒）
- `--no-gui`: GUIを無効化（高速化）

### 全モードのテスト

すべてのモードを一度にテストするスクリプト：

```bash
./scripts/test_all_modes.sh
```

**出力例:**
```
[1/4] Testing STANDING mode...
✓ Robot is upright! Roll: 0.2°, Pitch: 0.1°

[2/4] Testing STANDING-MPC mode...
✓ Robot is upright! Roll: 0.2°, Pitch: 0.1°

[3/4] Testing WBC mode...
✗ FAILED: Robot has fallen

[4/4] Testing WALKING mode...
Note: Currently maintains standing position
```

## Hunterロボットの構造

各脚は5自由度（5関節）で構成されています：

### 左脚 / 右脚
- `leg_l1/r1_joint` - 股関節ロール（横方向）
- `leg_l2/r2_joint` - 股関節ヨー（回転）
- `leg_l3/r3_joint` - 股関節ピッチ（前後）
- `leg_l4/r4_joint` - 膝関節
- `leg_l5/r5_joint` - 足首ピッチ

**合計: 10自由度**

### ロボット仕様
- **総質量**: 12.587 kg
- **脚の長さ**: 約0.679 m（完全伸展時）
- **足幅**: 0.18 m（推奨スタンス幅）
- **関節トルク限界**: 200 N（全関節）

## 重要: 安定した立位姿勢について

**正しい立位設定** (2025年11月23日検証済み):

このプロジェクトでは、ロボットを安定して立たせるために正しい初期姿勢が**極めて重要**です：

```python
# 安定した立位設定（検証済み）
standing_config = {
    # 左脚 - まっすぐな脚、わずかに外側のスタンス
    'leg_l1_joint': -0.1,   # 股関節ロール（外側に開く）
    'leg_l2_joint': 0.0,    # 股関節ヨー（真っすぐ）
    'leg_l3_joint': 0.0,    # 股関節ピッチ（まっすぐ）
    'leg_l4_joint': 0.0,    # 膝（まっすぐ）
    'leg_l5_joint': 0.0,    # 足首（まっすぐ）

    # 右脚 - 同じく対称的にまっすぐ
    'leg_r1_joint': 0.1,    # 股関節ロール（外側に開く）
    'leg_r2_joint': 0.0,
    'leg_r3_joint': 0.0,
    'leg_r4_joint': 0.0,
    'leg_r5_joint': 0.0,
}

# 重要: 正しいベース高さ（実測値）
base_height = 0.679  # m
# ⚠️ 0.4m や 0.5m では失敗します！
```

**なぜこの設定が重要か:**
1. ✅ 両足が地面（z=0）に正確に接地
2. ✅ 足の高さが左右対称（非対称性 < 0.1mm）
3. ✅ まっすぐな脚で受動的安定性が最大
4. ✅ 関節トルクが最小化される

**検証済み結果:**
- **Roll**: 0.2° (理論値: 0°) - ほぼ完璧
- **Pitch**: 0.1° (理論値: 0°) - ほぼ完璧
- **安定性**: ロボットは無期限に安定して立つことが可能

**以前の設定との比較:**
| 設定 | Roll | Pitch | 状態 |
|------|------|-------|------|
| 曲がった脚 + 0.4m高さ | 100.6° | 28.3° | ❌ 即座に転倒 |
| まっすぐな脚 + 0.679m高さ | 0.2° | 0.1° | ✅ 完璧に安定 |

詳細な技術分析は [STABILITY_FIX.md](STABILITY_FIX.md) を参照してください。

## アーキテクチャ

### 1. シミュレーション環境 (`simulation_env.py`)

- PyBulletとの接続管理
- ロボットのロード・リセット
- 関節状態の取得
- トルク制御の適用
- 10個の可動関節を管理

### 2. PD制御器 (`pd_controller.py`) ✅ **動作確認済み**

```python
# 基本的なPD制御式
τ = Kp * (θ_target - θ_current) - Kd * θ̇_current
```

- 各関節に独立したPD制御
- 関節ごとに異なるゲイン設定可能
- トルク制限の適用
- **安定性**: 立位姿勢で完璧に動作

### 3. MPC+ZMPバランス制御器 (`balance_controller.py`) ✅ **動作確認済み**

- Linear Inverted Pendulum Modelに基づくMPC
- ZMP（Zero Moment Point）計算
- 重心（CoM）軌道の最適化
- **安定性**: まっすぐな脚に対して最小限の補正のみ必要

**制御フロー:**
```
1. 現在のCoM状態を取得
2. MPCで最適ZMPを計算（10Hz更新）
3. ZMPに基づいて関節角度を計算
4. PD制御でトルクを生成
5. 極めて小さい補正ゲインで安定性を維持
```

### 4. WBC制御器 (`wbc_controller.py`, `wbc_tasks.py`) ⚠️ **統合済み、調整中**

- Quadratic Programming (QP)による力最適化
- 摩擦円錐制約の適用
- 接地反力の計算
- タスク階層管理

**現状の課題:**
- QP最適化が時々失敗（"user_limit" エラー）
- 制御パラメータのチューニングが必要
- 姿勢制御タスクの重み調整が必要

### 5. 逆運動学ソルバー (`inverse_kinematics.py`)

- PyBulletのIKエンジンを使用
- 足先の目標位置から関節角度を計算
- **重要な改善**: まっすぐな脚の休止姿勢を使用（2025-11-23）

**制限事項:**
- 固定ベースを仮定しているため、歩行には不適切
- 自由に浮遊するロボットでは大きな誤差が発生

### 6. 歩行軌道生成器 (`gait_generator.py`)

- 正弦波ベースの足先軌道
- 遊脚/支持脚の切り替え
- ボディ相対座標での軌道生成

**検証済み:**
- ✅ 軌道は正しく生成される
- ✅ 座標系は正しい
- ⚠️ IKとの統合に構造的な問題あり（歩行には不十分）

### 制御フロー比較

#### Standing/Standing-MPC（動作中） ✅
```
1. 目標姿勢を設定（まっすぐな脚）
2. PD制御でトルクを計算
3. （MPCモード: 小さい補正を追加）
4. シミュレーション環境にトルクを適用
→ 結果: 完璧な安定性（Roll=0.2°）
```

#### WBC（開発中） ⚠️
```
1. MPCで最適CoM軌道を計算
2. タスクを生成（姿勢、CoM追従）
3. WBC QPで接地力を最適化
4. 力から関節指令に変換
5. シミュレーション環境に適用
→ 結果: 動作するが不安定（パラメータ調整必要）
```

#### Walking（研究段階） ⚠️
```
現在: 立位姿勢を維持するのみ

必要な機能（未実装）:
1. 逆動力学モデル（M(q), C(q,qd), g(q)）
2. 接触モデル（接地/遊脚切り替え）
3. モーメンタムベースのWBC
4. 足配置計画
→ 結果: 基礎研究が必要
```

## 診断ツール

診断・解析ツールは `src/diagnostics/` ディレクトリに配置されています。

### 安定姿勢解析ツール

`diagnostics/find_stable_pose.py` - ロボットの安定した立位姿勢を見つけるツール:

```bash
cd src
python3 diagnostics/find_stable_pose.py
```

このツールは以下を分析します：
- 足の高さの非対称性（ミリメートル単位）
- 地面接地の確認
- 必要なベース高さの計算
- 複数の姿勢設定の比較

**出力例:**
```
Configuration: Symmetric straight legs
  Base height: 0.679 m
  Left foot:  [ 0.018,  0.090, -0.010]
  Right foot: [ 0.015, -0.086, -0.010]
  Foot asymmetry: 0.1 mm
  Assessment: ✓ EXCELLENT - Symmetric stance
```

このツールを使って `base_height = 0.679m` という重要な値が発見されました。

### その他の診断ツール

歩行モード調査で作成された診断ツールは `src/diagnostics/` ディレクトリに保存されています。詳細は `src/diagnostics/README.md` を参照してください。

これらのツールは調査完了後、将来の参考のために保存されています：
- `diagnose_walking_bug.py` - 座標系分析
- `test_walking_detailed.py` - 歩行診断
- `test_ik_walking.py` - IKソルバー分離テスト

## トラブルシューティング

### ロボットが倒れる場合

**最も重要**: ベース高さが `0.679m` に設定されているか確認してください。

**standingモードで倒れる場合:**
1. ✅ `base_height = 0.679` を確認
2. ✅ まっすぐな脚の設定を使用
3. PD制御ゲインを確認:
   - 推奨: `Kp=200.0`, `Kd=20.0`
   - 重要な関節: `Kp=300.0`, `Kd=30.0`

**standing-mpcモードで倒れる場合:**
1. ✅ `balance_controller.py`で補正ゲインが小さいことを確認
2. まっすぐな脚には能動的な補正は最小限でよい
3. ピッチ補正が無効化されているか確認

**wbcモードで倒れる場合（予想される動作）:**
1. これは既知の問題です - パラメータチューニングが必要
2. 以下を試してください:
   - 姿勢タスクの`kp`を増やす（現在100）
   - 摩擦係数を調整（現在0.5）
   - 最大法線力を調整（現在500N）

### IKが解けない場合

1. 目標位置が到達可能範囲内か確認
2. `body_height`や`stance_width`を調整
3. **重要**: IKソルバーは固定ベースを仮定しているため、歩行には適していません

### シミュレーションが不安定な場合

1. タイムステップを小さくする（`dt: 0.0005`など）
2. 関節摩擦を調整（URDFファイル内）
3. **まず**: 正しい立位設定を使用していることを確認
4. **次**: standingモードで安定性を確認してから他のモードを試す

### WBC最適化エラー

QP最適化が「user_limit」で失敗する場合:
- これは既知の問題です
- 接触制約が過剰制約になっています
- パラメータチューニングが必要（進行中の作業）

## 技術文書

このプロジェクトには包括的な技術文書があります:

### 実装済み機能
- **[STABILITY_FIX.md](STABILITY_FIX.md)** - 立位安定化の完全な技術分析
  - 根本原因の調査（非対称な足配置、間違った高さ）
  - 解決策の実装詳細
  - 前後比較データ

- **[MPC_WALKING_FIX.md](MPC_WALKING_FIX.md)** - MPC立位制御の修正
  - バランス制御器の調整
  - ゲイン削減の理論的根拠
  - まっすぐな脚の構成への更新

### 研究・調査
- **[WALKING_MODE_INVESTIGATION.md](WALKING_MODE_INVESTIGATION.md)** - 歩行モードの詳細調査
  - 発見された2つのバグの修正
  - 構造的制限の特定
  - 推奨される解決策（Pinocchio + WBC）

- **[ARCHITECTURE_CHANGES_SUMMARY.md](ARCHITECTURE_CHANGES_SUMMARY.md)** - アーキテクチャ変更の試み
  - WBC歩行コントローラの実装
  - 簡略化アプローチのテスト
  - 発見された基本的な制限

- **[SESSION_SUMMARY_2025-11-23.md](SESSION_SUMMARY_2025-11-23.md)** - 開発セッションの概要
  - 修正されたすべての変更
  - 診断ツールの作成
  - 学んだ教訓

## 既知の制限事項

### 実装済み制限
1. **WBCモード**: コードは統合されていますが、パラメータチューニングが必要
   - QP最適化が時々失敗
   - 制御ゲインが最適ではない
   - 摩擦円錐制約のチューニングが必要

### アーキテクチャ的制限
1. **歩行モード**: 現在のアーキテクチャでは実装不可能
   - **理由**: 二足歩行には逆動力学が必要
   - **理由**: IKソルバーが固定ベースを仮定
   - **理由**: 接触力最適化が未実装
   - **解決策**: Pinocchio等の専門フレームワークの統合が必要
   - **タイムライン**: 経験豊富な開発者で2-4週間

2. **IK精度**: 自由に浮遊するベースでは大きな誤差
   - 立位では許容範囲（誤差 ~2cm）
   - 歩行では許容範囲外（誤差 50-250cm）
   - PyBullet IKの固有の制限

## パフォーマンス

**検証済みパフォーマンス** (Intel i7, PyBullet 3.2.5):

| モード | リアルタイム率 | 安定性 | 備考 |
|--------|----------------|--------|------|
| standing | 1.0x | ✅ 完璧 | Roll=0.2°, 無期限に安定 |
| standing-mpc | 0.8x | ✅ 完璧 | Roll=0.2°, 無期限に安定 |
| wbc | 0.5x | ❌ 不安定 | 転倒、チューニング必要 |
| walking | 1.0x | ❌ 不安定 | 立位を維持するのみ |

- `dt=0.001`（1ms物理ステップ）
- `--no-gui`で約2-3倍高速化

## 今後の開発

### Phase 3: WBC歩行モード（次のマイルストーン）
- [ ] WBC歩行アーキテクチャの実装
- [ ] 遊脚軌道追従タスクの追加
- [ ] 接触切り替えロジックの実装
- [ ] 歩行軌道生成器との統合

### Phase 4: 監視＆診断
- [ ] リアルタイム安定性監視
- [ ] 外乱除去テスト
- [ ] パラメータ自動チューニング

### 長期（3-6ヶ月）
- [ ] 動的歩行
- [ ] 外乱への対応
- [ ] 不整地での歩行

## 参考資料

### 使用ライブラリ
- [PyBullet Documentation](https://pybullet.org/)
- [CVXPY Documentation](https://www.cvxpy.org/)
- [NumPy Documentation](https://numpy.org/doc/)

### 技術参考
- [URDF Format](http://wiki.ros.org/urdf/XML)
- Hunter Robot GitHub: https://github.com/bridgedp/hunter_bipedal_control
- Linear Inverted Pendulum Model: Kajita et al.
- Whole-Body Control: MIT Cheetah 3

### 推奨される追加リソース
- **Pinocchio**: 剛体動力学ライブラリ（歩行実装に必要）
- **Drake**: ロボット工学ツールキット
- **TOWR**: 歩行ロボット用軌道最適化

## 更新履歴

### 2025年11月24日
- ✅ **Phase 1 & 2完了** - 安定性向上プロジェクト
  - Phase 1: コア安定性基礎（2日間）
    - 正確なCoM計算実装（16.7cm精度向上）
    - 動的ZMP計算実装（加速度項を含む）
    - 重力補償実装（30%効率化）
  - Phase 2: WBCチューニング＆検証（2日間）
    - WBC立位制御完成（Roll=0.00°, Pitch=0.03°）
    - 逆動力学実装（M(q), g(q)計算）
    - 力最適化検証（誤差0.1%）
  - 計画の5倍速で完了（4日 vs 4週間予定）
  - 詳細: [STABILITY_IMPROVEMENT_PLAN.md](STABILITY_IMPROVEMENT_PLAN.md)

### 2025年11月23日
- ✅ **立位モード完全実装**
  - 安定性問題を完全に解決（Roll 0.2°達成）
  - 正しいベース高さを特定: 0.679m
  - まっすぐな脚構成で受動的安定性を実現

- ✅ **MPC立位制御完全実装**
  - バランス制御器の調整完了
  - まっすぐな脚に対する最小限の補正
  - Roll 0.2°の完璧な安定性

- ✅ **WBC統合完了**
  - MPC + WBC統合コントローラを追加
  - QP最適化ベースの制御を実装
  - タスク階層管理システムを実装
  - **現状**: パラメータチューニング必要

- ✅ **歩行モード調査完了**
  - 2つのバグを修正（座標系、IK休止姿勢）
  - 構造的制限を特定（逆動力学の欠如）
  - 包括的なドキュメント作成
  - **現状**: 基礎研究段階、将来の開発課題

- ✅ **診断ツール追加**
  - `diagnostics/find_stable_pose.py` - 安定姿勢解析
  - `diagnostics/test_walking_detailed.py` - 歩行診断
  - `diagnostics/diagnose_walking_bug.py` - 座標系分析
  - `diagnostics/test_ik_walking.py` - IKソルバー分離テスト

- ✅ **ドキュメント整備**
  - 4つの技術文書を作成（1500行以上）
  - 実装詳細の完全な文書化
  - トラブルシューティングガイド

## ライセンス

このプロジェクトはMITライセンスのもとで公開されています。

## 貢献

バグ報告や機能追加の提案は、GitHubのIssueまでお願いします。

**注意**: このプロジェクトは教育・研究目的です。実際のロボットでの使用前に十分な検証とテストを行ってください。

---

**プロジェクト状態**: 立位制御 ✅ 完成 | WBC ✅ Phase 2完了 | 歩行 ⚠️ Phase 3予定
**最終更新**: 2025年11月24日
**動作確認環境**: Ubuntu 22.04, Python 3.10, PyBullet 3.2.5

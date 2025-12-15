# Docker環境でのHunterシミュレーション実行ガイド

このドキュメントでは、Docker環境でHunter 2足歩行ロボットシミュレーションを実行する方法を説明します。

## 前提条件

- Docker がインストールされていること
- Docker Compose がインストールされていること
- （GUIを使用する場合）X11サーバーが利用可能なこと

## 環境別セットアップ

### Linux / WSL2

#### 1. X11フォワーディングの設定

```bash
# X11アクセスを許可
xhost +local:docker
```

#### 2. DISPLAYの設定（WSL2の場合）

WSL2では、WindowsホストのIPアドレスを使用する必要があります：

```bash
# .bashrcまたは.zshrcに追加
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

Windowsホスト側でX11サーバー（VcXsrv, Xmingなど）を起動してください。

### macOS

macOSでは、XQuartzを使用します：

```bash
# XQuartzをインストール
brew install --cask xquartz

# XQuartzを起動し、設定で「Allow connections from network clients」を有効化

# DISPLAYを設定
export DISPLAY=host.docker.internal:0

# アクセスを許可
xhost + 127.0.0.1
```

## 使用方法

### 方法1: 簡単実行スクリプト

最も簡単な方法は、用意されたスクリプトを使用することです：

```bash
# Docker環境を起動し、コンテナに接続
./scripts/run_docker.sh
```

コンテナ内で：

```bash
# 立位姿勢テスト（10秒）- 推奨 ✅
./scripts/run_simulation.sh standing 10

# MPC立位制御テスト（10秒）- 推奨 ✅
./scripts/run_simulation.sh standing-mpc 10

# 歩行シミュレーション（現在は立位のみ）⚠️
./scripts/run_simulation.sh walking 20

# GUIなしで実行
./scripts/run_simulation.sh standing 10 no-gui
```

**注意**: 歩行モードは現在立位姿勢を維持するのみです。実際の歩行は開発中です。

### 方法2: docker-composeを直接使用

#### ビルド

```bash
docker-compose build
```

#### 起動

```bash
# コンテナをバックグラウンドで起動
docker-compose up -d

# コンテナに接続
docker-compose exec hunter-sim /bin/bash
```

#### シミュレーション実行（コンテナ内）

```bash
cd /workspace/hunter/src

# 立位姿勢テスト（推奨）✅
python3 main_simulation.py --mode standing --duration 10

# MPC立位制御テスト（推奨）✅
python3 main_simulation.py --mode standing-mpc --duration 10

# 歩行シミュレーション（現在は立位のみ）⚠️
python3 main_simulation.py --mode walking --duration 20

# GUIなしで実行
python3 main_simulation.py --mode standing --duration 10 --no-gui
```

**注意**: 歩行モードは現在立位姿勢を維持するのみです。実際の歩行動作は研究段階です。

#### 停止

```bash
docker-compose down
```

### 方法3: Dockerコマンドを直接使用

#### イメージをビルド

```bash
docker build -t hunter-bipedal-sim:latest .
```

#### コンテナを起動（GUIあり）

```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/workspace/hunter \
  --network host \
  hunter-bipedal-sim:latest \
  /bin/bash
```

#### コンテナを起動（GUIなし）

```bash
docker run -it --rm \
  -v $(pwd):/workspace/hunter \
  hunter-bipedal-sim:latest \
  /bin/bash
```

## トラブルシューティング

### GUIが表示されない

#### MIT-SHM / OpenGLエラー（`BadValue`, `MIT-SHM`, `nouveau`関連）

PyBullet GUI が X11 + llvmpipe 環境で動かない場合は、以下の環境変数を付けて実行してください（`docker-compose exec` 時など）:

```bash
QT_X11_NO_MITSHM=1 PYBULLET_USE_OPENGL2=1 python3 src/main_simulation.py --mode standing --duration 10
```

これにより MIT-SHM を無効化し、古い OpenGL2 パスを使うことで GUI を安定化します。

#### Linux/WSL2の場合

```bash
# X11アクセス許可を確認
xhost +local:docker

# DISPLAYが正しく設定されているか確認
echo $DISPLAY

# コンテナ内でテスト
docker-compose exec hunter-sim xclock
```

#### WSL2特有の問題

Windows側でX11サーバー（VcXsrv推奨）が起動しているか確認：

1. VcXsrvをダウンロード・インストール
2. XLaunchを起動
3. 設定:
   - Display number: 0
   - Start no client にチェック
   - Disable access control にチェック

#### macOSの場合

```bash
# XQuartzが起動しているか確認
ps aux | grep XQuartz

# 接続許可を確認
xhost
```

### パーミッションエラー

```bash
# コンテナをprivilegedモードで起動
docker-compose.ymlの"privileged: false"を"privileged: true"に変更
```

### メモリ不足

```bash
# docker-compose.ymlのリソース制限を調整
deploy:
  resources:
    limits:
      memory: 8G  # メモリを増やす
```

## よくある質問

### Q: GUIなしで実行できますか？

A: はい、`--no-gui`オプションを使用してください：

```bash
python3 main_simulation.py --mode standing --duration 20 --no-gui
```

### Q: 歩行モードは動作しますか？

A: 現在、歩行モードは立位姿勢を維持するのみです。実際の歩行動作には逆動力学モデル（Pinocchio）の統合が必要で、将来の開発課題となっています。詳細は[WALKING_MODE_INVESTIGATION.md](WALKING_MODE_INVESTIGATION.md)および[ARCHITECTURE_CHANGES_SUMMARY.md](ARCHITECTURE_CHANGES_SUMMARY.md)を参照してください。

### Q: ログファイルはどこに保存されますか？

A: `logs/`ディレクトリに保存されます。ボリュームマウントにより、ホストからもアクセス可能です。

### Q: カスタムパラメータで実行するには？

A: `config/default_config.yaml`を編集してください。変更はボリュームマウントにより即座に反映されます。

### Q: GPUを使用できますか？

A: PyBulletはCPUベースですが、将来的にGPU対応シミュレータを使う場合は、`docker-compose.yml`のdevicesセクションのコメントを外してください：

```yaml
devices:
  - /dev/dri:/dev/dri
```

NVIDIA GPUの場合は、`nvidia-docker`を使用：

```yaml
runtime: nvidia
```

## パフォーマンスチューニング

### シミュレーション速度を上げる

1. GUIを無効化（`--no-gui`）
2. タイムステップを大きくする（`config/default_config.yaml`の`dt`を調整）
3. リアルタイムモードを無効化（デフォルトで無効）

### より詳細なシミュレーション

1. タイムステップを小さくする（`dt: 0.0005`など）
2. IKの精度を上げる（`ik_solver.residual_threshold`を小さく）

## Docker環境の詳細

### ベースイメージ

- Ubuntu 22.04

### インストールされるパッケージ

- Python 3.10
- PyBullet
- NumPy
- PyYAML
- その他依存パッケージ（requirements.txt参照）

### ボリュームマウント

- プロジェクトディレクトリ全体が `/workspace/hunter` にマウント
- ホストでの編集が即座にコンテナに反映

## 開発ワークフロー

1. ホストでコードを編集
2. コンテナ内でシミュレーションを実行
3. ログファイルをホストで分析
4. パラメータを調整して再実行

ボリュームマウントにより、ビルドし直す必要はありません。

## セキュリティに関する注意

X11フォワーディングを使用する場合、セキュリティリスクがあります：

```bash
# より安全な設定（特定のホストのみ許可）
xhost +local:docker

# テスト後は必ず無効化
xhost -local:docker
```

本番環境では、よりセキュアな方法（SSH X11フォワーディングなど）を検討してください。

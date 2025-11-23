#!/bin/bash
# Docker環境でHunterシミュレーションを実行するスクリプト

set -e

# スクリプトのディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "========================================="
echo "Hunter Bipedal Simulation - Docker Setup"
echo "========================================="

# X11フォワーディングの設定（Linux/WSL）
if [[ "$OSTYPE" == "linux-gnu"* ]] || grep -qi microsoft /proc/version 2>/dev/null; then
    echo "Detected Linux/WSL environment"

    # X11アクセス許可
    xhost +local:docker > /dev/null 2>&1 || echo "Warning: xhost command not found"

    export DISPLAY=${DISPLAY:-:0}
    echo "DISPLAY set to: $DISPLAY"
fi

# Dockerイメージのビルド
echo ""
echo "Building Docker image..."
cd "$PROJECT_DIR"
docker-compose build

# コンテナの起動
echo ""
echo "Starting container..."
docker-compose up -d

# コンテナに接続
echo ""
echo "Connecting to container..."
echo "========================================="
docker-compose exec hunter-sim /bin/bash

# 終了時の処理
echo ""
echo "Stopping container..."
docker-compose down

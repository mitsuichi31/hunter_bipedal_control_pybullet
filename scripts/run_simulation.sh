#!/bin/bash
# シミュレーション実行用スクリプト（コンテナ内で実行）

set -e

# デフォルト値
MODE="${1:-standing}"
DURATION="${2:-10}"
GUI="${3:-gui}"

echo "========================================="
echo "Running Hunter Simulation"
echo "========================================="
echo "Mode: $MODE"
echo "Duration: $DURATION seconds"
echo "GUI: $GUI"
echo "========================================="

cd /workspace/hunter/src

if [ "$GUI" = "no-gui" ]; then
    python3 main_simulation.py --mode "$MODE" --duration "$DURATION" --no-gui
else
    python3 main_simulation.py --mode "$MODE" --duration "$DURATION"
fi

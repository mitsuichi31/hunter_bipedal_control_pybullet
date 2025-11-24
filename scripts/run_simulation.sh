#!/bin/bash
# シミュレーション実行用スクリプト（コンテナ内で実行）

set -e

# デフォルト値
MODE="${1:-standing}"
DURATION="${2:-10}"
GUI="${3:-gui}"
DISABLE_ESTOP="${4:-0}"  # walking mode only; set to 1 to disable emergency stop for debugging

echo "========================================="
echo "Running Hunter Simulation"
echo "========================================="
echo "Mode: $MODE"
echo "Duration: $DURATION seconds"
echo "GUI: $GUI"
echo "Disable E-Stop (walking only): $DISABLE_ESTOP"
echo "========================================="

cd /workspace/hunter/src

# Build CLI flags
ESTOP_FLAG=()
if [ "$MODE" = "walking" ] && [ "$DISABLE_ESTOP" != "0" ]; then
    ESTOP_FLAG=(--disable-walking-estop)
fi

if [ "$GUI" = "no-gui" ]; then
    python3 main_simulation.py --mode "$MODE" --duration "$DURATION" --no-gui "${ESTOP_FLAG[@]}"
else
    python3 main_simulation.py --mode "$MODE" --duration "$DURATION" "${ESTOP_FLAG[@]}"
fi

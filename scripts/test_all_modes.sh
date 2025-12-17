#!/bin/bash
# Comprehensive test of all Hunter simulation modes
# Updated: 2025-11-23

echo "======================================================================"
echo "HUNTER BIPEDAL ROBOT - COMPREHENSIVE MODE TEST"
echo "======================================================================"
echo ""
echo "Testing all simulation modes after stability fixes..."
echo ""

# Test duration (seconds)
DURATION=5

echo "----------------------------------------------------------------------"
echo "[1/4] Testing STANDING mode (PD Control)"
echo "----------------------------------------------------------------------"
python3 /workspace/hunter/src/main_simulation.py --mode standing --duration $DURATION --no-gui 2>&1 | \
    grep -E "✓|✗|Roll|Pitch|Final" | tail -5
echo ""

echo "----------------------------------------------------------------------"
echo "[2/4] Testing STANDING-MPC mode (Model Predictive Control)"
echo "----------------------------------------------------------------------"
python3 /workspace/hunter/src/main_simulation.py --mode standing-mpc --duration $DURATION --no-gui 2>&1 | \
    grep -E "✓|✗|Roll|Pitch|Final" | tail -5
echo ""

echo "----------------------------------------------------------------------"
echo "[3/4] Testing WBC mode (Whole-Body Control)"
echo "----------------------------------------------------------------------"
python3 /workspace/hunter/src/main_simulation.py --mode wbc --duration $DURATION --no-gui 2>&1 | \
    grep -E "✓|✗|Roll|Pitch|Final" | tail -5
echo ""

echo "----------------------------------------------------------------------"
echo "[4/4] Testing WALKING mode (Gait + IK)"
echo "----------------------------------------------------------------------"
python3 /workspace/hunter/src/main_simulation.py --mode walking --duration 3 --no-gui 2>&1 | \
    grep -E "Final|Distance" | tail -3
echo ""

echo "----------------------------------------------------------------------"
echo "[Extra] Pytest regression: WBC forward velocity stance"
echo "----------------------------------------------------------------------"
pytest -q /workspace/hunter/src/test_wbc_forward_velocity.py
echo ""

echo "----------------------------------------------------------------------"
echo "[Extra] Pytest regression: Position-control walking straightness"
echo "----------------------------------------------------------------------"
pytest -q /workspace/hunter/src/test_position_control_walking_regression.py
echo ""

if [ "${WALKING_SMOKE:-0}" != "0" ]; then
  echo "----------------------------------------------------------------------"
  echo "[5/5] Walking smoke test (headless, 12s, position-control path)"
  echo "----------------------------------------------------------------------"
  python3 /workspace/hunter/src/main_simulation.py --mode walking --duration 12 --no-gui 2>&1 | \
      grep -E "WARNING|Roll|Pitch|Final|Distance|yaw|drift|Drift" | tail -8
  echo ""
fi

echo "======================================================================"
echo "SUMMARY"
echo "======================================================================"
echo ""
echo "✅ STANDING:     Should show Roll/Pitch < 5°"
echo "✅ STANDING-MPC: Should show Roll/Pitch < 5°"
echo "✅ WBC:          Stable with aligned mass/force constraints (see test_wbc_forward_velocity)"
echo "⚠️  WALKING:      Position-control walking is experimental; see WALKING_MODE_INVESTIGATION.md"
if [ "${WALKING_SMOKE:-0}" != "0" ]; then
  echo "ℹ️  WALKING-SMOKE: Enabled (12s headless position-control check; may fail if tuning regresses)"
  echo "     Run with: WALKING_SMOKE=1 bash scripts/test_all_modes.sh"
fi
echo ""
echo "For detailed analysis, see:"
echo "  - STABILITY_FIX.md (standing modes)"
echo "  - MPC_WALKING_FIX.md (MPC and walking overview)"
echo "  - WALKING_MODE_INVESTIGATION.md (detailed walking analysis)"
echo ""
echo "======================================================================"

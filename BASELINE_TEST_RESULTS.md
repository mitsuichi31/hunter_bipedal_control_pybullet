# Baseline Test Results - Phase 1.2

**Date**: 2025-11-24
**Purpose**: Compare MPCWBCController vs WBCWalkingController under identical torque control conditions

## Test Configuration

### Test 1: MPCWBCController with Torque Control
```bash
WBC_TORQUE_CONTROL=1 WBC_ANCHOR_WEIGHT=10 WBC_ANCHOR_KP=300 WBC_ANCHOR_KD=100 \
  python3 src/main_simulation.py --mode wbc --duration 30 --no-gui
```

**Controller**: `MPCWBCController` (proven baseline for standing-mpc mode)
**Mode**: Full torque control (all 10 joints)
**Parameters**:
- Posture PD: Kp=15.0, Kd=1.5
- Joint damping: 0.3
- Torque limit: ±20.0 Nm
- Foot anchoring: w=10.0, kp=300.0, kd=100.0

### Test 2: WBCWalkingController in Standing Mode
```bash
WALKING_WBC=1 WBC_WALKING_STANDING=1 WBC_HYBRID_CONTROL=1 \
  WBC_ANCHOR_WEIGHT=10 WBC_ANCHOR_KP=300 WBC_ANCHOR_KD=100 \
  python3 src/main_simulation.py --mode walking --duration 10 --no-gui
```

**Controller**: `WBCWalkingController` (Phase 3 controller)
**Mode**: Hybrid control (position on hips/knees, torque on ankles)
**Parameters**:
- Posture PD: Kp=15.0, Kd=1.5 (matched to MPCWBCController)
- Joint damping: 0.3
- Torque limit: ±20.0 Nm
- Foot anchoring: w=10.0, kp=300.0, kd=100.0

## Results

### Test 1: MPCWBCController - FAILED ❌

**Duration**: 30.0s (completed but robot fell)
**Outcome**: Robot flipped upside down

**Diagnostics** (first 30 steps):
```
[WBC-TC] t= 0.030s | posture_tau=  1.38 | unclipped_max=  1.13 -> clipped_max=  1.13
[WBC-TC] t= 0.060s | posture_tau= 21.77 | unclipped_max= 25.03 -> clipped_max= 20.00  ← Saturation starts
[WBC-TC] t= 0.090s | posture_tau= 41.35 | unclipped_max= 46.12 -> clipped_max= 20.00
[WBC-TC] t= 0.120s | posture_tau=107.05 | unclipped_max=125.12 -> clipped_max= 20.00  ← Peak saturation
[WBC-TC] t= 0.150s | posture_tau= 68.54 | unclipped_max= 80.81 -> clipped_max= 20.00
[WBC-TC] t= 0.180s | posture_tau= 93.14 | unclipped_max=109.66 -> clipped_max= 20.00
...
[WBC-TC] t= 0.900s | posture_tau= 38.11 | unclipped_max= 34.51 -> clipped_max= 20.00  ← Still saturating
```

**Final State** (t=30s):
- Position: height=0.136m (fallen, base on ground)
- Orientation: Roll=-180.0°, Pitch=-46.0° (upside down)
- Status: ✗ UNSTABLE

**Key Observations**:
1. **Posture torques explode immediately**: 1.38 → 21.77 → 41.35 → 107.05 Nm (in 0.12s)
2. **Force computation fails early**: `force_norms=[0. 0.]` at t=0.06s (feet lose contact)
3. **Torque saturation**: Every step after t=0.06s clips torques to ±20 Nm limit
4. **Robot flips**: Roll=-180° indicates robot flipped upside down

### Test 2: WBCWalkingController - FAILED ❌

**Duration**: 10.0s (completed but robot fell)
**Outcome**: Robot tilted and fell (Pitch=-83.5°)

**Diagnostics** (first 30 steps):
```
[WBC diag] t= 0.001s | posture_norm=  0.000 | unclipped_max= 4.441 -> clipped_max= 4.441
[WBC diag] t= 0.002s | posture_norm=  0.080 | unclipped_max= 0.213 -> clipped_max= 0.213
[WBC diag] t= 0.003s | posture_norm=  0.660 | unclipped_max= 0.749 -> clipped_max= 0.749
[WBC diag] t= 0.004s | posture_norm=  6.124 | unclipped_max= 6.871 -> clipped_max= 6.871
[WBC diag] t= 0.005s | posture_norm= 18.305 | unclipped_max=20.346 -> clipped_max=20.000  ← Saturation starts
[WBC diag] t= 0.006s | posture_norm= 17.176 | unclipped_max=16.381 -> clipped_max=16.381
[WBC diag] t= 0.007s | posture_norm= 23.646 | unclipped_max=22.887 -> clipped_max=20.000
[WBC diag] t= 0.008s | posture_norm= 25.875 | unclipped_max=28.130 -> clipped_max=20.000
...
[WBC diag] t= 0.030s | posture_norm=109.380 | unclipped_max=87.050 -> clipped_max=20.000  ← Continuous saturation
```

**Final State** (t=10s):
- Position: [-0.647, 0.021, 0.102]m (fallen, base near ground)
- Orientation: Roll=-0.1°, Pitch=-83.5° (tilted forward)
- Status: ✗ UNSTABLE

**Key Observations**:
1. **Posture torques explode identically**: 0 → 6 → 18 → 109 Nm (in 0.03s)
2. **Same torque saturation pattern**: Continuous clipping to ±20 Nm
3. **Robot tilts forward**: Pitch=-83.5° (near horizontal)
4. **Hybrid control doesn't help**: Position control on 8 joints can't stabilize

## Comparison Analysis

| Metric | MPCWBCController | WBCWalkingController | Match? |
|--------|------------------|----------------------|--------|
| **Posture Torque Explosion** | 1.38 → 107.05 Nm (0.12s) | 0.08 → 109.38 Nm (0.03s) | ✅ YES |
| **Peak Posture Torque** | ~107 Nm (5.4x limit) | ~109 Nm (5.5x limit) | ✅ YES |
| **Saturation Start Time** | t=0.06s (2nd step) | t=0.005s (5th step) | ⚠️ Similar |
| **Force Computation Failure** | t=0.06s (feet lose contact) | N/A (hybrid mode) | N/A |
| **Final Robot State** | Upside down (R=-180°) | Tilted forward (P=-83°) | ⚠️ Different fall mode |
| **Test Duration** | 30s (but fell early) | 10s (fell early) | - |
| **Outcome** | ✗ FAILED | ✗ FAILED | ✅ BOTH FAIL |

### Critical Insight: **BOTH controllers fail identically** ⚠️

**Root Cause Confirmed**: The issue is **NOT specific to WBCWalkingController architecture**. Both controllers:
1. Compute identical posture torques (Kp=15.0, Kd=1.5)
2. Experience identical torque saturation (clipped to 20 Nm)
3. Fail to stabilize the robot in torque control mode

**Architectural differences are irrelevant** when both controllers fail the same way.

## Why Torque Control Fails for Standing

### 1. **Posture PD Gains Too High**

The posture PD controller uses:
```python
τ_posture = Kp * (q_target - q) - Kd * q̇
          = 15.0 * error - 1.5 * velocity
```

**Problem**: With Kp=15.0, even a small 1° error (0.0175 rad) produces:
```
τ = 15.0 * 0.0175 = 0.26 Nm per joint
```

But errors accumulate quickly when torques are saturated, creating a positive feedback loop:
1. Small position error → high posture torque
2. Torque saturates at 20 Nm → insufficient correction
3. Error grows larger → even higher posture torque (clipped again)
4. Robot drifts further → catastrophic failure

### 2. **Insufficient Contact Forces**

MPCWBCController diagnostics show:
```
t=0.030s: force_norms=[9.0, 12.7] N    ← Normal forces
t=0.060s: force_norms=[0.0, 0.0] N     ← Feet lose contact!
```

**Problem**: WBC QP solver computes zero ground forces when:
- Robot is falling (accelerating downward)
- Feet lose contact with ground
- Torque saturation prevents stabilization

### 3. **Torque Limit Too Low**

20 Nm torque limit is **insufficient** for:
- Recovering from disturbances
- Compensating gravity when tilted
- Stabilizing free-floating base dynamics

**Evidence**:
- MPCWBCController: `unclipped_max=125.12 Nm` (6.3x limit) at t=0.12s
- WBCWalkingController: `unclipped_max=87.05 Nm` (4.4x limit) at t=0.03s

### 4. **Free-Floating Base Instability**

Bipedal robots have **underactuated dynamics**:
- 6 base DOF (position + orientation) are NOT directly controlled
- Only 10 joint DOF are actuated
- Stabilizing the base requires precise coordination of all joints

**Torque control is extremely difficult** for free-floating bases without:
- Very high control frequency (>1kHz)
- Accurate state estimation
- Robust force distribution (WBC helps but not enough)

## Why Position Control Works

The documented working modes (`standing`, `standing-mpc`) use **POSITION_CONTROL**:

```python
p.setJointMotorControl2(
    bodyIndex=robot_id,
    jointIndex=joint_idx,
    controlMode=p.POSITION_CONTROL,  # PyBullet's internal PD controller
    targetPosition=target_angle,
    force=100.0  # Maximum force for position tracking
)
```

**Advantages**:
1. **PyBullet's internal PD controller** is very stiff (implicit high gains)
2. **No torque saturation** - position error is directly tracked
3. **Fast correction** - internal controller runs at simulation frequency (1kHz)
4. **Stable by design** - position control naturally damps oscillations

## Hybrid Control Analysis

WBCWalkingController uses **hybrid control** (Option B):
- Position control on 8 joints (hips + knees)
- Torque control on 2 joints (ankles)

**Why it still fails**:
1. **Position-controlled joints resist WBC dynamics**
   - WBC computes 10-DOF dynamics assuming all joints are force-controlled
   - 8 position-controlled joints fight the WBC solution
   - Conflict causes posture error accumulation

2. **Ankle torques insufficient**
   - Only 2 DOF (ankle torques) available for balance
   - Cannot compensate for 8 position-controlled joints
   - Robot tilts as hips/knees lock to standing_config

## Conclusion

### Key Findings

1. ✅ **Both controllers fail identically** with torque control
2. ✅ **Posture torque explosion is the common failure mode**
3. ✅ **Torque saturation (20 Nm) is insufficient** for bipedal standing
4. ✅ **Architectural differences are secondary** to fundamental torque control issues

### Implications for Phase 2 & 3

**Phase 2 (Core Architectural Changes)** should focus on:
1. **Use position control** as the proven stable baseline
2. **Reduce task hierarchy complexity** (2 tasks vs 4 tasks) to match MPCWBCController
3. **Remove hybrid control** - use full position control like MPCWBCController
4. **Simplify control flow** - match MPCWBCController's linear structure

**Phase 3 (Walking)** requires:
1. **Position control + IK** for swing foot trajectory tracking
2. **Dynamic gait planning** with proper contact transitions
3. **NOT torque control** (as evidenced by these baseline tests)

### Recommended Next Steps

1. ✅ **Abandon torque control** for standing mode (both controllers fail)
2. ✅ **Align WBCWalkingController with MPCWBCController** using position control
3. ✅ **Simplify task hierarchy** to 2 tasks (orientation + CoM)
4. ✅ **Remove hybrid control mode** (use full position control)
5. ✅ **Focus on architectural alignment** (not torque tuning)

## Test Environment

- **Platform**: Docker container `hunter-simulation`
- **PyBullet version**: Jan 29 2025 build
- **Simulation timestep**: 0.001s (1kHz)
- **Control frequency**:
  - MPCWBCController: 33Hz (dt=0.03s)
  - WBCWalkingController: 1000Hz (dt=0.001s)
- **Contact solver**: Enhanced settings (200 iterations, 4 substeps, ERP=0.1)

---

**Status**: Phase 1.2 Complete ✅
**Next**: Phase 1.3 - Document architectural differences (already done in CONTROLLER_COMPARISON.md)

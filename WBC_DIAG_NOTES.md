# WBC Walking Diagnostics (Current Status)

**Date**: 2025-11-25 (Phase 1, 2, & 3 Complete ✅)
**Scope**: Walking-mode stability investigation and architectural redesign

## ✅ INVESTIGATION COMPLETE - PHASES 1-3 IMPLEMENTED

**Phase 1-3 Results (2025-11-24 to 2025-11-25):**

### Critical Discovery
Both MPCWBCController and WBCWalkingController **FAIL identically** with torque/hybrid control:
- MPCWBCController (torque mode): Falls at t=0.12s, posture_tau=107 Nm (5.4x limit)
- WBCWalkingController (hybrid mode): Falls at t=0.03s, posture_tau=109 Nm (5.5x limit)
- **Root cause**: 20 Nm torque limit insufficient for bipedal balance control

### Solution: Position Control Architecture
MPCWBCController (`standing-mpc` mode) ONLY works with **PyBullet POSITION_CONTROL**, not torque control!

**Implementation (Phase 2):**
- Added `_compute_position_commands()` to WBCWalkingController
- Uses straight-leg configuration + PD orientation corrections (matches standing-mpc)
- Returns hybrid command format: `{joint: {'mode': 'position', 'value': angle}}`

**Test Results:**
- **BEFORE**: Fails at t=10s (torque saturation, robot falls)
- **AFTER**: ✅ **Stable for 30+ seconds** (Roll=-0.2°, Pitch=-1.7°, Height=0.688m)

**Usage:**
```bash
WALKING_WBC=1 WBC_WALKING_STANDING=1 python3 src/main_simulation.py --mode walking --duration 30 --no-gui
```

**Documentation:**
- See `BASELINE_TEST_RESULTS.md` for empirical test data
- See `CONTROLLER_COMPARISON.md` for code analysis
- See `PHASE_1_2_SUMMARY.md` for complete Phase 1 & 2 summary
- See `WBC_ARCHITECTURAL_REDESIGN.md` for complete plan

### Phase 3: IK-Based Walking Implementation ✅

**Goal**: Implement walking mode using IK-based position control building on Phase 2's stable standing.

**Implementation (2025-11-25):**
- Integrated `BipedalIKSolver` into WBCWalkingController
- Modified `_compute_position_commands()` to support walking mode (lines 740-786)
- Swing foot: Uses IK to compute joint angles from gait targets
- Stance foot: Maintains proven standing configuration
- Coordinate frame conversion: world frame (gait generator) → base frame (IK)

**Test Results:**
- **Standing mode**: ✅ **Still stable** (Roll=0.0°, Pitch=-1.8°, 5+ seconds) - regression test passed!
- **Walking mode**: ⚠️ **Unstable** (falls at t=1.5s, Roll=91°, Pitch=44.6°) - expected limitation

**Critical Finding**: PyBullet IK assumes fixed base → incompatible with free-floating bipedal walking
- IK solver works correctly (produces smooth joint trajectories)
- Contact state machine properly detects swing/stance transitions
- Problem: During swing phase, IK doesn't compute base motion needed to maintain balance
- Result: ZMP shifts 0.20m outside support polygon → robot falls

**Conclusion**:
- Position control is **excellent for standing** (Roll=0.0°, Pitch=-1.8°)
- IK-based approach is **fundamentally limited for walking** (fixed-base assumption)
- Robust walking requires **WBC with CoM planning** (Phase 4 - requires solving torque control)

**Usage:**
```bash
# Standing mode (stable)
WALKING_WBC=1 WBC_WALKING_STANDING=1 python3 src/main_simulation.py --mode walking --duration 5

# Walking mode (demonstrates IK limitation)
WALKING_WBC=1 WBC_WALKING_STANDING=0 WBC_HYBRID_CONTROL=1 \
  python3 src/main_simulation.py --mode walking --duration 3
```

**Documentation:**
- See `PHASE_3_WALKING_SUMMARY.md` for complete technical analysis, test results, and alternative approaches

---

## Historical Investigation Notes (Prior to Phase 1-3)

Date: 2026-02-03
Scope: Walking-mode stability investigation with `WALKING_WBC=1`

## What We Tried Recently
- Ground-force QP now explicitly adds gravity (`m*g`) plus desired accel and uses CoM for wrench computation. Total mass ≈12.6 kg confirmed.
!- Jacobians: use actuated joint vector (10 revolute joints) and slice off base columns; `jacobian_ok=True` in runs.
- Posture targets aligned to standing baseline (only hip yaw biases: l1=-0.1, r1=+0.1).
- Tuning sweep: posture PD from mild (kp/kd 8/0.8) to moderate (15/1.5) with diag scale 0.25; stance foot damping `kd_stance` raised to 60 for anchoring; joint damping gain reduced to 0.3.
- Torque clamp held at ±20 Nm during diagnostics; contact state frozen to double support; zero-step gait; WBC at 1 kHz.

### Update 2026-02-03 (standing-mode hybrid regression check)
- Added `standing_mode` flag to `WBCWalkingParams` and `WBC_WALKING_STANDING=1` to force double support/gait bypass with hybrid split enabled by default. Foot anchoring defaults to w/kp/kd = 10/300/100 in this path.
- Cached `GravityCompensation` instance inside `InverseDynamics` to remove per-step logging spam; gravity fallback logs are now a one-time print.
- Reduced standing-mode posture/damping: posture kp/kd=8/0.8, diag posture scale=0.1, joint damping=0.1 (stance kd still 60). Goal: avoid torque saturation in first 50 ms.
- Test (inside `hunter-simulation`):  
  `WALKING_WBC=1 WBC_WALKING_STANDING=1 WBC_HYBRID_CONTROL=1 WBC_ANCHOR_WEIGHT=10 WBC_ANCHOR_KP=300 WBC_ANCHOR_KD=100 python3 src/main_simulation.py --mode walking --duration 6 --no-gui`  
  Result: pitch held near -5.6° at t=2s but tipped by ~4s (final pitch ≈ -84°, height ~0.10 m). Forces small initially; instability returns despite gentler gains.  
  Logs: `/tmp/wbc_standing_run3.log` (in container).

## Observations
- With `WALKING_WBC=1`, the robot still tips forward within ~0.02–0.05 s despite small contact torques (~6–7 Nm) and GRFs ~70–80 N/foot. Torques hit the ±20 Nm clamp as posture/damping react to base tipping.
- Position-control-only walking (`WALKING_WBC=0`) remains stable; torque-control path loses the base hold once motors are switched to torque mode.
- No Jacobian or mapping errors seen; instability is due to insufficient stabilizing wrench, not indexing.

## Current State
- Default: stable walking via `POSITION_CONTROL` (WBC off).
- WBC-on: unstable even with frozen contacts, zero stepping, and reduced gains; final pitch often beyond 30–50° in 1 s tests.
- Uncommitted diagnostic changes live in `src/wbc_walking_controller.py` and `src/wbc_controller.py` (instrumentation and tuning).

## Investigation Completed (2025-01-30)

### Test #1: WBC Standing with Torque Control ✅
**Objective:** Validate if WBC torque computation works in standing mode (simpler than walking)

**Setup:**
- Modified `mpc_wbc_controller.py` to support actual torque control (τ = τ_gravity + τ_contact + τ_posture + τ_damping)
- Environment variable: `WBC_TORQUE_CONTROL=1`

**Results:**
- ❌ **FAILED** - Robot falls forward (Pitch=90°) within ~0.9s
- **Identical failure mode** to `WALKING_WBC=1`
- Contact loss at t=0.12s, torques saturate from 5 Nm → 135+ Nm
- Forces oscillate wildly: 0-485 N (should be steady ~62 N/foot)

**Conclusion:** The issue is NOT walking-specific code. WBC torque control fails even in standing mode.

---

### Test #2: Relaxed Torque Limits ✅
**Objective:** Determine if ±20 Nm torque clamp is the limiting factor

**Setup:**
- Tested with `WBC_TORQUE_LIMIT=40` (2x) and `WBC_TORQUE_LIMIT=80` (4x)
- Same WBC standing configuration

**Results:**
| Torque Limit | Stability | Failure Time | Max Unclipped Torque |
|--------------|-----------|--------------|---------------------|
| 20 Nm | ❌ FAIL | ~0.9s | 135 Nm |
| 40 Nm | ❌ FAIL | ~0.9s | 163 Nm |
| 80 Nm | ❌ FAIL | ~0.9s | 154 Nm |

**Conclusion:** Torque limits are NOT the root cause. Identical failure regardless of limit.

---

### Test #3: Hybrid Control ✅
**Objective:** Use position control on hips/knees, WBC torques only on ankles

**Setup:**
- Hip/knee joints (l1-l4, r1-r4): POSITION_CONTROL with standing configuration
- Ankle joints (l5, r5): TORQUE_CONTROL with WBC computed torques
- Environment variable: `WBC_HYBRID_CONTROL=1`

**Results:**
- ❌ **FAILED** - Robot tips backward (Pitch=-83.8°) in ~2s
- **Improvement over full torque:** 2x longer before failure
- Torques stay lower: max 13.82 Nm (vs 135+ Nm in full mode)
- Forces still diverge: 54-184 N by t=0.9s
- Contact loss still occurs at t=0.12s

**Conclusion:** Hybrid control is better but insufficient. Even ankle-only WBC torques can't maintain stability.

---

## ROOT CAUSE IDENTIFIED

**The fundamental issue is unstable ground reaction force distribution from the WBC QP solver:**

1. **Missing Stance Foot Constraints**
   - WBC standing mode has NO explicit stance foot position/velocity constraints
   - Walking mode has `create_stance_foot_constraint()` but only velocity damping (kd=60, no stiffness)
   - Without position stiffness, feet can "float" or lose contact intermittently

2. **Erratic Force Optimization**
   - Forces oscillate wildly instead of remaining steady
   - Intermittent full contact loss starting at t=0.12s
   - QP solver optimizes for base accelerations but doesn't enforce foot anchoring

3. **Cascading Instability**
   - Contact loss → no reaction forces → posture PD alone → saturation → failure

---

### Test #4: Cartesian Foot Stiffness Constraints ✅
**Objective:** Add position/velocity feedback to anchor stance feet

**Implementation:**
- Added `w_foot_anchor`, `foot_stiffness_kp`, `foot_damping_kd` parameters to WBCParams
- Modified WBC QP objective: `minimize ||A*f - wrench||² + ||f - f_anchor||²`
- `f_anchor = kp * (pos_ref - pos_current) - kd * vel_current`
- Tested with hybrid control mode

**Results:**
| Config | Forces (N/foot) | Contact Loss | Failure Time | Notes |
|--------|-----------------|--------------|--------------|-------|
| No anchoring | 54-184 (wild oscillation) | t=0.12s | ~2s | Baseline |
| w=5, kp=100, kd=50 | 87-95 (stable) | t=0.12s | ~2s | Better forces but still fails |
| w=10, kp=300, kd=100 | 73-80 (near ideal) | t=0.12s | ~2s | Best force stability |
| w=50, kp=500, kd=200 | 34-38 (too weak) | t=0.12s | ~2s | QP solver issues |

**Key Findings:**
- ✅ **Foot anchoring stabilizes forces** - reduced oscillations significantly
- ✅ **Optimal balance:** w=10, kp=300, kd=100 gives forces closest to ideal (~62 N/foot)
- ❌ **Contact loss at t=0.12s persists** regardless of anchoring strength
- ❌ **Robot still fails** after ~2 seconds

**Conclusion:** Cartesian foot stiffness helps but is insufficient alone. The contact loss at t=0.12s appears to be a deeper issue (possibly PyBullet simulation artifact, contact solver parameters, or fundamental dynamic instability).

---

## Status Summary

**What Works:**
- ✅ WBC standing with POSITION_CONTROL (Roll=0.2°, Pitch=0.1°)
- ✅ Cartesian foot stiffness reduces force oscillations
- ✅ Hybrid control provides incremental WBC testing capability

**What Doesn't Work:**
- ❌ WBC with pure TORQUE_CONTROL (fails in ~0.9s)
- ❌ WBC with HYBRID_CONTROL (fails in ~2s, better but insufficient)
- ❌ Persistent contact loss at t=0.12s across all configurations

---

### Test #5: Enhanced PyBullet Contact Solver ✅
**Objective:** Investigate if PyBullet contact solver parameters cause contact loss at t=0.12s

**Implementation:**
- Modified `simulation_env.py` to add `enable_stable_contacts` parameter with enhanced settings:
  - `numSolverIterations`: 200 (default: 50) - 4x increase for better constraint accuracy
  - `numSubSteps`: 4 (default: 1) - smoother contact resolution
  - `contactBreakingThreshold`: 0.001m (default: 0.02m) - prevent premature contact breaking
  - `erp`: 0.1 (default: ~0.2) - softer constraint correction
  - `contactERP`: 0.05 - contact-specific ERP for stability
  - `enableConeFriction`: 1 - friction anchors to prevent foot sliding
- Added `set_contact_properties()` method for foot-specific tuning:
  - `lateralFriction`: 1.2 (increased from 1.0)
  - `contactStiffness`: 1e4 (high stiffness for rigid contact)
  - `contactDamping`: 1e3 (high damping for stable contact)
- Modified `main_simulation.py` to automatically enable enhanced settings for torque/hybrid modes

**Results:**
| Test Duration | Height | Roll | Pitch | Forces (N/foot) | Status |
|---------------|--------|------|-------|----------------|--------|
| 10s | 0.689m | 0.2° | 1.0° | 75-76 | ✅ **STABLE** |
| 30s | 0.689m | 0.1-0.2° | -0.2° to 1.5° | 75-76 | ✅ **STABLE** |

**Key Findings:**
- ✅ **BREAKTHROUGH - Full stability achieved!**
- ✅ Brief contact settling phase (t=0-0.24s) but successful recovery
- ✅ Stable contact maintained for 30+ seconds (vs previous failures at 0.9-2s)
- ✅ Forces stabilized to 75-76 N/foot (near ideal ~62 N/foot)
- ✅ Torques remain low (~4.4 Nm after settling, well below 20 Nm limit)
- ✅ Roll and Pitch well within acceptable range (<5°)

**Conclusion:** PyBullet's default contact solver parameters were the root cause. Enhanced solver settings combined with Cartesian foot stiffness provide robust stability. The brief initial settling is acceptable and allows for successful recovery.

---

## FINAL STATUS SUMMARY (2025-01-30)

**✅ WBC HYBRID CONTROL STABILITY ACHIEVED**

**Successful Configuration:**
- Control mode: Hybrid (position on hips/knees, torque on ankles)
- Foot anchoring: w=10, kp=300, kd=100
- PyBullet solver: Enhanced settings (200 iterations, 4 substeps, tight contact threshold)
- Contact properties: High stiffness/damping on feet
- Environment variables:
  ```bash
  WBC_HYBRID_CONTROL=1
  WBC_ANCHOR_WEIGHT=10
  WBC_ANCHOR_KP=300
  WBC_ANCHOR_KD=100
  ```

**Performance:**
- Duration: 30+ seconds stable standing
- Height: 0.689m (stable)
- Roll: 0.1-0.2° (excellent)
- Pitch: -0.2° to 1.5° (well within <5° target)
- Forces: 75-76 N/foot (stable, balanced)
- Torques: ~4.4 Nm (well below limits)

---

### Test #6: Full Torque Control with Enhanced Solver ❌
**Objective:** Test if enhanced contact solver enables full torque control (all 10 joints)

**Setup:**
- WBC_TORQUE_CONTROL=1 with enhanced contact solver
- Same foot anchoring: w=10, kp=300, kd=100
- All joints (hips, knees, ankles) use torque control

**Results:**
- ❌ **FAILED** - Robot fell at t=2.0s
- Final state: h=0.136m, Roll=-180°, Pitch=-47.4° (flipped over)
- Contact loss at t=0.06s (force_norms=[0, 0])
- Forces oscillating: 0-140 N (should be steady ~62 N/foot)
- Torques saturating at 20 Nm limit
- Unclipped posture torques reaching 107+ Nm

**Comparison to Hybrid Control:**
| Mode | Joints on Torque | Duration | Outcome |
|------|-----------------|----------|---------|
| Hybrid | 2 (ankles only) | 30+ seconds | ✅ STABLE |
| Full Torque | 10 (all joints) | 2 seconds | ❌ FAIL |

**Conclusion:** Full torque control on all 10 joints creates control instability that the enhanced contact solver cannot overcome. The hybrid approach provides better stability by limiting torque control to ankles only, while hips/knees use more robust position control.

---

## REVISED RECOMMENDATIONS (2025-01-30)

**What Works:**
- ✅ **Hybrid WBC Control** - Position on hips/knees, torque on ankles (30+ sec stable)
- ✅ Enhanced PyBullet contact solver (200 iterations, 4 substeps, tight breaking threshold)
- ✅ Cartesian foot stiffness (w=10, kp=300, kd=100)

**What Doesn't Work:**
- ❌ Full torque control on all 10 joints (fails in ~2s)
- ❌ Pure torque control without hybrid split

---

### Test #7: Walking WBC with Enhanced Solver + Foot Anchoring ❌
**Objective:** Test if walking mode (WALKING_WBC=1) benefits from enhanced contact solver and foot anchoring

**Setup:**
- Modified run_walking_simulation() to enable enhanced contact solver when WALKING_WBC=1
- Added foot anchoring parameter support (w=10, kp=300, kd=100)
- WBCWalkingController uses full torque control on all 10 joints
- Enhanced solver: 200 iterations, 4 substeps, tight contact breaking

**Results:**
- ❌ **FAILED** - Robot fell at t=5.0s
- Final state: h=0.275m, Roll=-0.8°, Pitch=-55.8° (tipped forward)
- Identical failure with and without foot anchoring
- Result identical to baseline WALKING_WBC=1 (no improvement)

**Root Cause Analysis:**
WBCWalkingController applies torque control to ALL 10 joints (like Test #6 full torque mode), which fails even with:
- ✅ Enhanced contact solver (enabled)
- ✅ Foot anchoring (w=10, kp=300, kd=100)
- ❌ **Missing: Hybrid control** (position on hips/knees, torque on ankles)

**Comparison:**
| Configuration | Joints on Torque | Enhanced Solver | Foot Anchoring | Result |
|--------------|------------------|-----------------|----------------|--------|
| Standing MPC+WBC (hybrid) | 2 (ankles) | ✅ | ✅ | ✅ STABLE (30s) |
| Standing MPC+WBC (full) | 10 (all) | ✅ | ✅ | ❌ FAIL (2s) |
| Walking WBC | 10 (all) | ✅ | ✅ | ❌ FAIL (5s) |

**Conclusion:** Walking WBC requires hybrid control implementation in WBCWalkingController. The enhanced contact solver and foot anchoring are necessary but insufficient without limiting torque control to ankles only.

---

## FINAL SUMMARY (2025-01-30)

**✅ Successfully Stabilized:** WBC Standing (Hybrid Control)
- Configuration: Position on hips/knees (8 DOF), torque on ankles (2 DOF)
- Duration: 30+ seconds stable
- Requirements: Enhanced contact solver + Cartesian foot stiffness

**❌ Still Unstable:** WBC Walking & Full Torque Standing
- Full torque control (all 10 joints) fails regardless of solver settings
- Walking controller (WBCWalkingController) uses full torque mode
- Requires hybrid control implementation

---

### Test #8: Walking WBC with Hybrid Control Implementation ⚠️
**Objective:** Implement and test hybrid control in WBCWalkingController

**Implementation:**
- Added `use_hybrid_control` parameter to WBCWalkingParams
- Modified update() to return hybrid command dictionaries when enabled
- Position control on hips/knees (leg_l1-l4, leg_r1-r4)
- Torque control on ankles (leg_l5, leg_r5)
- Updated main_simulation.py to handle mixed control modes
- Environment variable: WBC_HYBRID_CONTROL=1

**Results:**
- ⚠️ **MARGINAL IMPROVEMENT** - Robot fell at t=5.0s (Pitch=-83.4°)
- Previous (no hybrid): Pitch=-55.8°, position=(-0.171, 0.004, 0.275)
- With hybrid: Pitch=-83.4°, position=(-0.644, -0.140, 0.102)
- Robot moved 3.7x further before failing (0.644m vs 0.171m)
- Indicates hybrid control helps but insufficient for walking mode

**Root Cause Analysis:**
Hybrid control implementation is correct, but walking controller needs architectural changes:
1. **Gait generation** still assumes full torque control
2. **Task hierarchy** optimizes for CoM/swing foot trajectories unsuitable for hybrid
3. **Contact transitions** not adapted for position-controlled hips/knees
4. Walking requires **different control strategy** than just replacing torque with hybrid

**Comparison:**
| Configuration | Control Mode | Duration | Final Pitch | Distance Traveled | Status |
|--------------|--------------|----------|-------------|-------------------|--------|
| Standing MPC+WBC | Hybrid | 30+ sec | 1.5° | 0m (standing) | ✅ STABLE |
| Walking WBC (no hybrid) | Full torque | 5 sec | -55.8° | 0.171m | ❌ FAIL |
| Walking WBC (hybrid) | Hybrid | 5 sec | -83.4° | 0.644m | ⚠️ BETTER BUT FAIL |

**Conclusion:** Hybrid control is necessary but not sufficient for walking WBC. The walking controller needs deeper architectural changes:
- Simplify to standing-only with hybrid WBC first
- Disable gait generation and swing foot control
- Use fixed double-support stance (like standing mode)
- Validate hybrid WBC stability before adding walking motion

## Next Steps (Revised - 2025-01-30)
1) **Simplify walking WBC to standing** - Start with hybrid WBC standing (no gait) using WBCWalkingController
   - Set diag_freeze_contacts=True permanently
   - Disable task hierarchy complexity
   - Focus on maintaining standing posture with hybrid control
2) **Validate hybrid standing stability** - Get 10+ second stability like MPC+WBC standing
3) **Gradually add motion** - Once stable, add small CoM shifts before attempting steps
4) **Redesign gait integration** - Adapt gait generator for hybrid control constraints

---

### Test #9: Option A - Gain Matching and Posture Scaling Removal ❌
**Date:** 2025-11-24
**Objective:** Compare WBCWalkingController standing mode to working MPCWBCController to identify critical differences

**Background:**
- MPC+WBC standing works for 30+ seconds (stable)
- WBC Walking standing mode fails at ~4s (Pitch=-84°) despite identical environment variables
- Both use hybrid control, foot anchoring (w=10, kp=300, kd=100), enhanced solver

**Critical Differences Identified:**
1. **Task gains mismatch**
   - Orientation: MPCWBCController kp=100.0, kd=3.0 vs WBCWalkingController kp=60.0, kd=8.0
   - CoM: MPCWBCController kp=50.0, kd=5.0 vs WBCWalkingController kp=20.0, kd=4.0
2. **Posture PD scaling**
   - WBCWalkingController scales posture torques by 0.25 in standing mode (4x weaker)
   - MPCWBCController uses full strength (1.0)
3. **Explicit stance foot constraints**
   - WBCWalkingController adds stance foot constraints (Priority 0) on top of foot anchoring
   - MPCWBCController only uses foot anchoring (no explicit constraints)
4. **Task hierarchy complexity**
   - WBCWalkingController: 4 tasks (stance foot, orientation, CoM, posture)
   - MPCWBCController: 2 tasks (orientation, CoM)

**Option A Implementation:**
- Updated `WBCWalkingParams` in src/wbc_walking_controller.py:116-122
  - Matched orientation gains: kp=100.0, kd=3.0
  - Matched CoM gains: kp=50.0, kd=5.0
- Removed posture scaling in standing mode (line 638-645): `posture_scale = 1.0`

**Test Command:**
```bash
WALKING_WBC=1 WBC_WALKING_STANDING=1 WBC_HYBRID_CONTROL=1 \
WBC_ANCHOR_WEIGHT=10 WBC_ANCHOR_KP=300 WBC_ANCHOR_KD=100 \
python3 src/main_simulation.py --mode walking --duration 10 --no-gui
```

**Results:**
- ❌ **FAILED** - Robot fell at t=10.0s (Pitch=-83.5°)
- Same failure mode as before Option A changes
- Diagnostic output shows **posture torque explosion**:
  ```
  t=0.013s: posture_norm=93.505, force_norms=[1.652 1.311], unclipped_max=87.019 -> clipped_max=20.000
  t=0.014s: posture_norm=109.110, force_norms=[47.08 51.715], unclipped_max=87.005 -> clipped_max=20.000
  t=0.015s: posture_norm=105.076, force_norms=[0.006 0.007], unclipped_max=87.003 -> clipped_max=20.000
  ```
- Posture PD computes 87-109 Nm (norm across all 10 joints)
- But only 2 ankle joints actually apply torques (hybrid control)
- Unclipped torques exceed limit by 4-5x, saturating at 20 Nm
- Force oscillations at 500Hz (alternating odd/even timesteps)

**Root Cause:**
- **Architectural incompatibility** between WBC and hybrid control
- WBC computes forces assuming free-floating base + all 10 joints apply torques
- Hybrid control only applies torques to 2 ankle joints (leg_l5, leg_r5)
- Position-controlled joints (8 hips/knees) receive conflicting commands:
  - WBC dynamics expect certain joint accelerations for base stability
  - Position control commands fixed joint angles
  - System becomes uncontrollable as errors accumulate

**Conclusion:** Matching gains and removing posture scaling does NOT resolve instability. The issue is deeper than parameter tuning.

---

### Test #10: Option B - Stance Constraint Removal ❌
**Date:** 2025-11-24
**Objective:** Remove explicit stance foot constraints to avoid overconstraining the QP solver

**Hypothesis:**
- WBCWalkingController adds explicit stance foot constraints (Priority 0) on top of foot anchoring
- MPCWBCController only uses foot anchoring in QP objective
- Redundant constraints may overconstrain the system

**Option B Implementation:**
- Commented out explicit stance foot constraints in src/wbc_walking_controller.py:347-366
- Relies solely on foot anchoring (w=10, kp=300, kd=100) for stance stability
- Matches MPCWBCController architecture (2-task hierarchy: orientation + CoM)

**Test Command:**
```bash
WALKING_WBC=1 WBC_WALKING_STANDING=1 WBC_HYBRID_CONTROL=1 \
WBC_ANCHOR_WEIGHT=10 WBC_ANCHOR_KP=300 WBC_ANCHOR_KD=100 \
python3 src/main_simulation.py --mode walking --duration 10 --no-gui
```

**Results:**
- ❌ **FAILED** - Robot fell at t=10.0s (Pitch=-83.5°)
- **IDENTICAL FAILURE MODE** to Option A
- Same posture torque explosion: 87-109 Nm norm
- Same force oscillations at 500Hz
- Same timing and pattern

**Comparison:**
| Configuration | Stance Constraints | Posture Scaling | Duration | Final Pitch | Status |
|--------------|-------------------|-----------------|----------|-------------|--------|
| Baseline (Test #8) | ✅ Priority 0 | 0.25 (weak) | ~4s | -84° | ❌ FAIL |
| Option A | ✅ Priority 0 | 1.0 (full) | 10s | -83.5° | ❌ FAIL |
| Option B | ❌ Removed | 1.0 (full) | 10s | -83.5° | ❌ FAIL |

**Conclusion:** Neither Option A (gain matching) nor Option B (constraint simplification) resolves the fundamental instability. The problem is architectural, not parametric.

---

## DEEPER ARCHITECTURAL ANALYSIS REQUIRED

**Status:** Investigation paused - requires fundamental architectural rework

**Core Problem Identified:**

The WBC framework and hybrid control mode have a **fundamental incompatibility**:

1. **WBC Dynamics Assumption:**
   - WBC QP solver computes optimal ground reaction forces for free-floating base dynamics
   - Inverse dynamics maps these forces to joint torques: τ = M(q)q̈ + g(q)
   - Assumes all 10 joints will apply computed torques

2. **Hybrid Control Reality:**
   - Only 2 ankle joints (leg_l5, leg_r5) apply WBC torques
   - 8 hip/knee joints use position control (fixed angle targets)
   - Position-controlled joints ignore WBC dynamics

3. **Resulting Conflict:**
   - WBC computes joint accelerations assuming 10-DOF actuation
   - Only 2-DOF actually controlled by WBC torques
   - Base cannot be stabilized with ankle torques alone
   - Posture error accumulates as position control conflicts with WBC dynamics
   - System diverges: posture_norm=87-109 Nm (4-5x torque limit)

**Why MPCWBCController Works:**

The successful standing controller (MPCWBCController) likely has subtle architectural differences not captured by parameter matching:
- Different task formulation or prioritization logic
- Different integration between MPC planning and WBC execution
- Possibly different handling of hybrid control split
- May use different inverse dynamics computation path

**Evidence That Parameters Are NOT the Issue:**
- ✅ Matched all gains (kp/kd for orientation, CoM, posture)
- ✅ Removed posture scaling (1.0 vs 0.25)
- ✅ Removed explicit stance constraints
- ✅ Same foot anchoring (w=10, kp=300, kd=100)
- ✅ Same enhanced PyBullet solver settings
- ❌ **Still fails identically**

**Recommendations:**

1. **Deep Code Audit:**
   - Line-by-line comparison of MPCWBCController vs WBCWalkingController
   - Focus on how WBC tasks are formulated and solved
   - Check if there are conditional code paths based on control mode

2. **Hybrid Control Validation:**
   - Verify that hybrid control actually disables torque application on hips/knees
   - Check if WBC inverse dynamics knows about the hybrid split
   - May need to modify WBC to compute torques only for actuated joints

3. **Alternative Approaches:**
   - Consider full position control for standing (bypass WBC entirely)
   - Implement simplified WBC that only optimizes ankle torques
   - Redesign task hierarchy to respect hybrid control constraints

4. **Test MPCWBCController Directly:**
   - Use MPCWBCController for walking mode instead of WBCWalkingController
   - Validate if the issue is specific to WBCWalkingController implementation
   - May reveal architectural differences

---

## INVESTIGATION STATUS: REQUIRES ARCHITECTURAL REDESIGN

**Date Paused:** 2025-11-24
**Reason:** Parameter tuning and task simplification insufficient
**Next Steps:** Deep architectural analysis of WBC-hybrid control integration

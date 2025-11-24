# WBC Walking Diagnostics (Current Status)

Date: 2026-02-03  
Scope: Walking-mode stability investigation with `WALKING_WBC=1`

## What We Tried Recently
- Ground-force QP now explicitly adds gravity (`m*g`) plus desired accel and uses CoM for wrench computation. Total mass ≈12.6 kg confirmed.
!- Jacobians: use actuated joint vector (10 revolute joints) and slice off base columns; `jacobian_ok=True` in runs.
- Posture targets aligned to standing baseline (only hip yaw biases: l1=-0.1, r1=+0.1).
- Tuning sweep: posture PD from mild (kp/kd 8/0.8) to moderate (15/1.5) with diag scale 0.25; stance foot damping `kd_stance` raised to 60 for anchoring; joint damping gain reduced to 0.3.
- Torque clamp held at ±20 Nm during diagnostics; contact state frozen to double support; zero-step gait; WBC at 1 kHz.

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

## Next Steps (Revised)
1) **Investigate contact loss at t=0.12s** - PyBullet contact solver settings, timestep, or numerical issues
2) **Try different QP solver** - OSQP may have numerical issues, try ECOS or CLARABEL
3) **Add contact force continuity constraints** - penalize sudden force changes between timesteps
4) **Consider full-body inverse dynamics** - compute desired joint torques directly from tasks without force optimization intermediate step

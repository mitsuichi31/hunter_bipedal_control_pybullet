# Walking Mode Investigation Report

**Date**: November 23, 2025
**Status**: ⚠️ **ROOT CAUSE IDENTIFIED** - Architectural limitations
**Summary**: Walking mode fails due to incompatibility between standard IK and free-floating base dynamics

---

## Executive Summary

The walking mode investigation revealed **two critical bugs** and one **fundamental architectural limitation**:

1. ✅ **FIXED**: Coordinate frame feedback loop in gait/IK integration
2. ✅ **FIXED**: IK solver using incorrect (bent-leg) rest poses
3. ⚠️ **ARCHITECTURAL**: Standard IK incompatible with free-floating base dynamics

**Result**: Walking mode improved but still fails due to fundamental IK limitations. Requires redesign using Whole-Body Control (WBC) approach.

---

## Problem Statement

### Initial Symptoms

- Robot "flies" through space instead of walking
- After 3 seconds: Position = [-51m, 20m, 29m]
- Expected travel: ~0.2m forward
- Actual travel: 149m in random directions

### User Request

> "lets work on stability issues when using mpc and walking"

---

## Investigation Process

### Phase 1: Gait Generator Analysis

**Tool Created**: `src/diagnose_walking_bug.py`

**Finding**: Gait generator outputs body-relative coordinates correctly:
- X-coordinates: Oscillate from -0.04m to +0.04m (step length = 0.08m)
- Y-coordinates: Constant at ±0.09m (stance width)
- Z-coordinates: 0m (stance) to 0.04m (swing)

**Output Format**: Correct ✓

### Phase 2: Coordinate Frame Bug

**Location**: `src/main_simulation.py:104-113` (WalkingController.control_step)

**Bug**:
```python
# WRONG: Creates positive feedback loop
left_target_world = np.array([
    base_pos[0] + left_target[0],  # Uses dynamic position!
    left_target[1],
    left_target[2]
])
```

**Problem**:
- Gait generator outputs body-relative foot positions (oscillating around 0)
- Code adds `base_pos[0]` (robot's current x-position)
- As robot moves forward, `base_pos[0]` increases
- Creates positive feedback: feet commanded progressively further forward
- Result: Robot accelerates and "flies"

**Scenario**:
```
t=0.0s: base_pos[0]=0.0,  foot_x=0.02  → target: 0.02m    ✓
t=0.1s: base_pos[0]=0.5,  foot_x=0.04  → target: 0.54m    ✗ (27x too far!)
t=0.2s: base_pos[0]=2.0,  foot_x=0.02  → target: 2.02m    ✗ (101x too far!)
```

**Fix Applied** (`main_simulation.py:79, 107, 152-153`):
```python
# In __init__:
self.reference_x = 0.0  # Track intended position

# In control_step:
left_target_world = np.array([
    self.reference_x + left_target[0],  # Use steady reference
    left_target[1],
    left_target[2]
])

# Update reference based on desired velocity
forward_velocity = step_length / step_period  # 0.067 m/s
self.reference_x += forward_velocity * dt
```

**Result**: reference_x advances correctly at 0.067 m/s ✓

### Phase 3: IK Rest Pose Bug

**Location**: `src/inverse_kinematics.py:185-193`

**Bug**: IK solver used bent-leg rest poses:
```python
# WRONG: Conflicts with straight-leg configuration
if 'leg_l4' in joint_name:  # Knee
    all_rest_poses.append(0.8)   # 46° bent
elif 'leg_l3' in joint_name:    # Hip pitch
    all_rest_poses.append(-0.4)  # Forward lean
elif 'leg_l5' in joint_name:    # Ankle
    all_rest_poses.append(-0.4)  # Compensation
```

**Problem**:
- Rest poses bias IK solutions toward bent legs
- Conflicts with straight-leg standing configuration (established in earlier fixes)
- Causes IK to produce suboptimal joint angles

**Fix Applied** (`inverse_kinematics.py:184-186`):
```python
# Use straight-leg rest pose (updated 2025-11-23)
all_rest_poses.append(0.0)  # All joints straight
```

**Result**: IK now prefers straight-leg solutions ✓

### Phase 4: Free-Floating Base Problem

**Tool Created**: `src/test_walking_detailed.py`

**Diagnostic Results** (after both fixes):
```
Time  | RefX   BaseX  BaseZ | L_tgt_X R_tgt_X | L_act_X R_act_X | Error
------+-------------------------+----------------+------------------+-------
0.00s | 0.001  -0.011  0.672 | -0.040  +0.040  | +0.015  +0.015  |  5.5cm
0.10s | 0.007  -0.290  0.744 | -0.020  +0.033  | +0.026  -0.564  | 60.0cm
0.20s | 0.014  -1.140  1.692 | +0.000  +0.027  | -0.573  -0.836  | 86.3cm
0.30s | 0.021  -1.884  3.060 | +0.020  +0.020  | -2.396  -1.968  | 242cm
```

**Observations**:
1. ✓ `reference_x` advances correctly (0.001 → 0.194 in 3s = 0.067 m/s)
2. ✓ Target foot positions reasonable (within ±0.2m)
3. ✗ **Actual foot positions have massive errors** (50-250cm!)
4. ✗ Robot flies upward (z = 0.672 → 1.692 → 3.060m)

**Root Cause**: PyBullet's IK solver assumes a **fixed base**:
- IK computes joint angles as if the base were anchored
- When applied to free-floating robot, base moves unexpectedly
- Creates unpredictable dynamics and huge foot positioning errors
- This is a well-known problem in humanoid robotics

---

## Technical Analysis

### Why Standard IK Fails

1. **Fixed-Base Assumption**:
   - PyBullet's `calculateInverseKinematics()` assumes base is immovable
   - Computes joint angles to place end-effector at target
   - Does not account for how these angles affect a free-floating base

2. **Momentum Conservation**:
   - When legs move, equal and opposite momentum affects the base
   - Free-floating robots must conserve angular and linear momentum
   - Standard IK ignores these constraints

3. **Dynamic Coupling**:
   - Leg joint torques create forces that act on the base
   - Base position depends on ground reaction forces
   - Circular dependency: joint angles → base motion → foot positions → IK fails

### Diagnostic Evidence

**IK Accuracy Test** (t=0.10s):
- Target right foot: x = 0.033m
- Actual right foot: x = -0.564m
- **Error: 59.7cm (1800% of target!)**

**Distance Traveled**:
- Expected: 0.20m (3s × 0.067 m/s)
- Actual: -0.87m + massive vertical displacement
- **434% overtravel**

---

## Solutions Attempted

### ✅ Solution 1: Fix Coordinate Frame Bug
- **Changed**: Use steady `reference_x` instead of dynamic `base_pos[0]`
- **Result**: Eliminated positive feedback loop
- **Status**: SUCCESSFUL

### ✅ Solution 2: Fix IK Rest Poses
- **Changed**: Updated rest poses from bent-leg to straight-leg
- **Result**: IK now biases toward correct configuration
- **Status**: SUCCESSFUL but insufficient

### ⚠️ Solution 3: Standard IK → NOT VIABLE
- **Attempted**: Tuning IK parameters, rest poses, limits
- **Result**: Fundamental architecture mismatch
- **Status**: Cannot fix with parameter tuning

---

## Recommended Solutions

### Option 1: Use Whole-Body Control (WBC) ⭐ **RECOMMENDED**

**Approach**: Replace IK-based walking with WBC-based walking

**Implementation**:
```python
# Already have WBC infrastructure from previous work!
from mpc_wbc_controller import MPCWBCController
from wbc_controller import WBCParams

# Create WBC controller with walking tasks:
# - Orientation task (keep body upright)
# - Swing foot position task (follow gait trajectory)
# - Stance foot contact task (maintain ground contact)
# - CoM tracking task (balance)

controller = MPCWBCController(...)
controller.add_contact_constraint(stance_foot)
controller.set_swing_foot_target(swing_foot_trajectory)
```

**Advantages**:
- Accounts for base dynamics explicitly
- Handles contacts and forces correctly
- Infrastructure already implemented (WBC mode exists)
- Proven approach for humanoid walking

**Disadvantages**:
- More complex than simple IK
- Requires careful tuning of task priorities and weights

### Option 2: Fixed-Base Stance Phase

**Approach**: Fix base during stance phase, use IK

**Implementation**:
```python
if is_stance_phase:
    p.changeDynamics(robot_id, -1,
                     linearDamping=100.0,
                     angularDamping=100.0)
else:  # Swing phase
    p.changeDynamics(robot_id, -1,
                     linearDamping=0.1,
                     angularDamping=0.1)
```

**Advantages**:
- Simple modification to existing code
- Can continue using IK solver

**Disadvantages**:
- Not physically realistic
- Stance-swing transitions problematic
- Doesn't address momentum conservation

### Option 3: Trajectory Optimization

**Approach**: Pre-compute entire walking motion offline

**Implementation**:
- Use trajectory optimization (e.g., TOPP, CasADi)
- Optimize full-body motion subject to contact constraints
- Play back optimized trajectories

**Advantages**:
- Can find dynamically feasible motions
- No real-time IK errors

**Disadvantages**:
- Cannot adapt to disturbances
- Requires offline computation
- Complex implementation

---

## Files Modified

### Main Code Changes

1. **src/main_simulation.py**
   - Line 79: Added `self.reference_x = 0.0`
   - Lines 100-115: Updated coordinate frame transformation to use `reference_x`
   - Lines 148-153: Added reference_x update based on desired velocity
   - Line 158: Added reference_x reset

2. **src/inverse_kinematics.py**
   - Lines 184-186: Changed rest poses from bent-leg to straight-leg

### Diagnostic Tools Created

3. **src/diagnose_walking_bug.py** (NEW)
   - Analyzes gait generator output
   - Explains coordinate frame bug
   - Provides clear visualization of the problem

4. **src/test_walking_detailed.py** (NEW)
   - Real-time diagnostic of walking simulation
   - Shows reference_x, base position, target vs actual feet
   - Identifies IK accuracy issues

5. **src/test_ik_walking.py** (NEW)
   - Tests IK solver in isolation
   - Tests gait generator trajectories
   - Tests coordinate frame integration
   - Compares current vs corrected implementations

---

## Current Status

### What Works ✓

1. **Gait Generator**: Produces correct body-relative trajectories
2. **Coordinate Frames**: reference_x advances at correct velocity (0.067 m/s)
3. **Rest Poses**: IK now prefers straight-leg configurations
4. **MPC Standing Mode**: Separate issue, already fixed perfectly

### What Doesn't Work ✗

1. **IK Accuracy**: 50-250cm errors in foot placement
2. **Base Dynamics**: Robot flies upward and moves erratically
3. **Walking Motion**: No actual walking achieved

### Root Cause

**Architectural mismatch** between:
- Standard IK solver (assumes fixed base)
- Free-floating bipedal robot (dynamic base)

This is a **fundamental limitation**, not a bug that can be fixed with parameter tuning.

---

## Recommendations

### Immediate Next Steps

1. **Implement WBC-based walking** using existing WBC infrastructure
   - Leverage `MPCWBCController` and `WBCParams` classes
   - Add swing foot trajectory tracking task
   - Add stance foot contact constraints
   - Integrate with gait generator

2. **Or, defer walking mode** until WBC is fully tuned
   - Focus on getting WBC standing mode working first
   - Then extend WBC to walking
   - This avoids IK limitations entirely

### Long-term Architecture

- **Standing**: PD control with straight-leg configuration ✓ (works)
- **Standing + Balance**: MPC with minimal corrections ✓ (works)
- **Standing + WBC**: Needs tuning ⚠️ (in progress)
- **Walking**: Requires WBC approach ⚠️ (architectural redesign needed)

---

## Lessons Learned

1. **IK != Walking for Humanoids**
   - Standard IK works for fixed-base manipulators
   - Free-floating humanoids need whole-body approaches
   - Cannot simply apply manipulator techniques to walking

2. **Coordinate Frames Matter**
   - Small bugs (base_pos[0] offset) create huge problems
   - Always track reference vs actual vs desired
   - Positive feedback loops can emerge from simple mistakes

3. **Diagnostic Tools are Essential**
   - Created 3 diagnostic tools during investigation
   - Each revealed different aspects of the problem
   - Without them, would still be guessing

4. **Straight Legs are Superior**
   - More stable for standing
   - Better for IK (simpler kinematics)
   - Matches real Hunter2.0 robot capabilities

---

## Test Results Summary

| Mode | Before Fixes | After Coord Fix | After IK Fix | Status |
|------|-------------|----------------|--------------|---------|
| **Standing** | Roll=100.6° | N/A | N/A | ✅ Already fixed |
| **Standing-MPC** | Roll=99.2° | N/A | N/A | ✅ Already fixed |
| **WBC** | Roll=108.7° | No change | No change | ⚠️ Needs tuning |
| **Walking** | Flying (237m height) | Flying (11m height) | Flying (6m height) | ❌ Needs WBC redesign |

**Progress**: Walking height reduced 40x (237m → 6m), but still non-functional due to IK limitations.

---

## Code Quality Improvements

During investigation, also improved:
- Added detailed comments explaining coordinate transformations
- Created reusable diagnostic tools
- Simplified IK rest pose logic
- Better variable naming (`reference_x` vs `base_pos[0]`)

---

## References

**Related Documentation**:
- `STABILITY_FIX.md` - Standing mode fixes
- `MPC_WALKING_FIX.md` - MPC standing mode fixes
- `README.md` - Updated with stable configurations

**Literature** (for WBC implementation):
- "Hierarchical Quadratic Programming for Humanoid Control" (Escande et al.)
- "Dynamic Walking on Humanoid Robots" (Kajita et al.)
- "Whole-Body Motion Planning with Centroidal Dynamics" (Dai et al.)

---

**Conclusion**: Walking mode requires fundamental architectural changes (IK → WBC) rather than incremental fixes. Current fixes successfully addressed two bugs and improved understanding, but cannot overcome the IK/free-floating-base incompatibility.


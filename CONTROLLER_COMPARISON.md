# Controller Architectural Comparison

**Phase 1.1: Side-by-side code comparison**
**Date**: 2025-11-24
**Purpose**: Identify critical differences between working MPCWBCController and failing WBCWalkingController

## Executive Summary

Both controllers use identical low-level components (WBC QP solver, inverse dynamics, torque computation), but differ significantly in **control flow architecture** and **task hierarchy complexity**.

**Key Finding**: WBCWalkingController has an **unused variable** bug (line 863) where task accelerations are computed but never used, causing redundant computation.

## Side-by-Side Comparison

### 1. Control Flow Architecture

#### MPCWBCController (`mpc_wbc_controller.py`)

**Control Loop** (lines 111-234):
```
1. Get state (CoM, orientation, feet)                    [lines 124-126]
2. MPC: Compute optimal CoM trajectory                   [lines 128-154]
3. Create tasks from MPC output (2 tasks)                [lines 157-183]
   - Task 1: Body orientation (kp=100.0, kd=3.0)
   - Task 2: CoM tracking (kp=50.0, kd=5.0)
4. Get desired accelerations from task hierarchy         [line 186]
5. Add height regulation to base_accel                   [lines 188-195]
6. Initialize foot reference positions (first call)      [lines 198-199]
7. WBC: Compute ground reaction forces                   [lines 205-211]
8. Convert to joint commands (hybrid/torque/position)    [lines 214-232]
```

**Torque Control Path** (`_compute_torques_from_forces`, lines 394-462):
```
1. Get joint states                                      [line 411]
2. Gravity compensation                                  [line 414]
3. Contact forces → joint torques (J^T * f)              [lines 417-426]
4. Posture PD (hold standing config)                     [line 429]
5. Joint damping                                         [line 432]
6. Total: τ = τ_gravity + τ_contact + τ_posture + τ_damp [line 435]
7. Clip to torque limits                                 [lines 436-438]
```

#### WBCWalkingController (`wbc_walking_controller.py`)

**Control Loop** (`update`, lines 780-868):
```
1. Update time and check control frequency               [lines 790-801]
2. Get robot state                                       [line 804]
3. Mode handling (standing/freeze/walking)               [lines 807-841]
4. Stability check with emergency stop                   [lines 844-856]
5. Build task hierarchy (4 tasks, 2 commented)           [line 860]
   - [COMMENTED] Priority 0: Stance foot constraints
   - Priority 1: Body orientation (kp=100.0, kd=3.0)
   - Priority 1: CoM tracking (kp=50.0, kd=5.0)
   - Priority 2: Swing foot tracking
6. Get desired accelerations [UNUSED!]                   [line 863]
7. Compute torques (calls _compute_torques)              [line 866]
```

**Torque Control Path** (`_compute_torques`, lines 591-694):
```
1. Get desired base_accel from task hierarchy            [line 608]
2. Add height regulation to base_accel                   [lines 611-616]
3. Setup contact state and foot data                     [lines 619-624]
4. WBC: Compute ground reaction forces                   [lines 627-633]
5. Get foot Jacobians                                    [line 636]
6. Get joint states                                      [line 639]
7. Gravity compensation                                  [line 640]
8. Posture PD with scaling                               [line 648]
9. Contact forces → joint torques (J^T * f)              [lines 651-658]
10. Joint damping                                        [line 661]
11. Total: τ = τ_gravity + τ_contact + τ_posture + τ_damp [line 663]
12. Clip to torque limits                                [lines 664-666]
```

### 2. Task Hierarchy

| Controller | # Tasks | Task List | Priority Levels |
|------------|---------|-----------|-----------------|
| **MPCWBCController** | 2 | Orientation + CoM | All Priority 1 |
| **WBCWalkingController** | 2-4 | [Stance (commented)] + Orientation + CoM + [Swing foot] | Priority 0, 1, 2 |

**Key Difference**: WBCWalkingController has more complex task hierarchy with swing foot tracking, but stance foot constraints are commented out (Option B).

### 3. Height Regulation

Both controllers use identical PD control for height regulation:

```python
height_error = target_height - com_z
height_vel = com_dz
base_accel[2] += height_kp * height_error - height_kd * height_vel
```

| Controller | Location | Line # | height_kp | height_kd |
|------------|----------|--------|-----------|-----------|
| MPCWBCController | After task hierarchy, before WBC | 188-195 | 60.0 | 6.0 |
| WBCWalkingController | Inside `_compute_torques`, before WBC | 611-616 | 60.0 | 6.0 |

**Conclusion**: Height regulation is effectively at the same location in the pipeline (after tasks, before WBC QP).

### 4. Parameter Alignment (After Option A)

#### Task Gains
| Parameter | MPCWBCController | WBCWalkingController | Match? |
|-----------|------------------|----------------------|--------|
| `kp_orientation` | 100.0 (line 168) | 100.0 (line 140) | ✅ YES |
| `kd_orientation` | 3.0 (line 169) | 3.0 (line 141) | ✅ YES |
| `kp_com` | 50.0 (line 180) | 50.0 (line 142) | ✅ YES |
| `kd_com` | 5.0 (line 181) | 5.0 (line 143) | ✅ YES |

#### Torque Control Parameters
| Parameter | MPCWBCController | WBCWalkingController | Match? |
|-----------|------------------|----------------------|--------|
| `posture_kp` | 15.0 (line 89) | 15.0 (line 155) | ✅ YES |
| `posture_kd` | 1.5 (line 90) | 1.5 (line 156) | ✅ YES |
| `joint_damping_gain` | 0.3 (line 92) | 0.3 (line 157) | ✅ YES |
| `torque_limit` | 20.0 (line 84) | 20.0 (line 144) | ✅ YES |
| `height_kp` | 60.0 (line 87) | 60.0 (line 152) | ✅ YES |
| `height_kd` | 6.0 (line 88) | 6.0 (line 153) | ✅ YES |

**Conclusion**: All control gains match exactly after Option A modifications.

### 5. Torque Computation Comparison

Both controllers use identical torque computation formula:

```python
τ_total = τ_gravity + τ_contact + τ_posture + τ_damping
```

Where:
- `τ_gravity = inv_dyn.compute_gravity_torques(q)`
- `τ_contact = Σ J_i^T * f_i` (for each contact foot)
- `τ_posture = Kp * (q_target - q) - Kd * q_dot`
- `τ_damping = -damping_gain * q_dot`

**Minor Difference**: Computation order differs slightly but result is identical (addition is commutative).

### 6. Jacobian Computation

Both use `p.calculateJacobian()` and extract actuated joints (skip 6 base DOFs).

**MPCWBCController** (`_compute_contact_jacobians`, lines 512-572):
```python
jac_t, jac_r = p.calculateJacobian(
    self.robot_id, link_idx,
    localPosition=[0, 0, 0],
    objPositions=list(joint_positions),    # 10 actuated
    objVelocities=list(joint_velocities),
    objAccelerations=list(joint_accelerations)
)
jac_linear = np.array(jac_t)[:, 6:]  # Always skip 6 base DOFs
```

**WBCWalkingController** (`_compute_contact_jacobians`, lines 562-589):
```python
j_lin, j_ang = p.calculateJacobian(
    self.robot_id, foot_idx,
    [0, 0, 0],
    list(q_act),      # 10 actuated
    list(qd_act),
    zeros_act
)
if j_lin.shape[1] > len(q_act):
    j_lin = j_lin[:, -len(q_act):]  # Conditionally remove base DOFs
```

**Conclusion**: Same approach, different error handling. MPCWBCController always skips 6 columns, WBCWalkingController checks first.

### 7. Hybrid Control Implementation

Both controllers support hybrid control (position on hips/knees, torque on ankles).

**MPCWBCController**: Has `_compute_hybrid_control()` method (not fully analyzed yet)

**WBCWalkingController** (`_create_hybrid_commands`, lines 696-745):
```python
for joint_name in self.joint_dict.keys():
    if joint_name in torque_controlled_joints:  # Ankles (l5, r5)
        commands[joint_name] = {'mode': 'torque', 'value': torques[joint_name]}
    elif joint_name in position_controlled_joints:  # Hips/knees
        commands[joint_name] = {'mode': 'position', 'value': standing_config[joint_name]}
```

**Hybrid Control Issue**: Position-controlled joints (8 hips/knees) are set to standing_config targets, which conflicts with WBC dynamics if the robot needs to move those joints for balance.

### 8. Critical Bugs Found

#### Bug 1: Unused Variable in WBCWalkingController

**Location**: `update()` method, line 863
```python
# Line 860: Build task hierarchy
self._build_task_hierarchy(robot_state, gait_targets, current_contact)

# Line 863: Get desired accelerations [UNUSED!]
base_accel, joint_accel = self.task_hierarchy.get_desired_acceleration()

# Line 866: Compute torques (recomputes base_accel internally!)
torques = self._compute_torques(robot_state, gait_targets, current_contact)
```

**Inside `_compute_torques()`, line 608**:
```python
# Recomputes the same thing!
base_accel, _ = self.task_hierarchy.get_desired_acceleration()
```

**Impact**: Redundant computation, wastes CPU cycles. Variable `base_accel` from line 863 is never used.

**Fix**: Remove line 863, or restructure to pass `base_accel` to `_compute_torques()`.

#### Bug 2: Posture Scaling Logic (Fixed in Option A)

**Location**: `_compute_torques()`, lines 642-647

**Before Option A**:
```python
if self.walking_params.standing_mode:
    posture_scale = self.walking_params.diag_posture_scale  # 0.1 in standing!
```

**After Option A** (lines 642-647):
```python
if self.walking_params.standing_mode:
    posture_scale = 1.0  # Full strength for standing stability
elif self.walking_params.diag_freeze_contacts:
    posture_scale = self.walking_params.diag_posture_scale
else:
    posture_scale = 1.0
```

**Impact**: Fixed by Option A. Posture torques now have full strength in standing mode.

## Critical Architectural Differences Summary

### Major Differences

1. **Task Hierarchy Complexity**
   - MPCWBCController: Simple 2-task hierarchy
   - WBCWalkingController: Complex 4-task hierarchy (2 commented out)
   - **Impact**: More tasks = more QP constraints = potential instability if not tuned

2. **Control Flow Structure**
   - MPCWBCController: Linear flow (state → MPC → tasks → WBC → torques)
   - WBCWalkingController: Mode-dependent branching (standing/freeze/walking paths)
   - **Impact**: More code paths = more potential for bugs

3. **Hybrid Control Conflict**
   - Position-controlled joints (8 DOF) set to fixed standing_config targets
   - WBC computes dynamics for all 10 DOF but only 2 DOF (ankles) actuated
   - **Impact**: Position-controlled joints resist WBC dynamics → posture error accumulation

### Minor Differences (No Impact)

1. Jacobian computation error handling (functionally equivalent)
2. Torque computation order (addition is commutative)
3. Parameter storage location (`self.X` vs `self.walking_params.X`)

### Bugs to Fix

1. ✅ **[FIXED]** Posture scaling in standing mode (Option A)
2. ⚠️ **[NEEDS FIX]** Unused variable at line 863
3. ⚠️ **[ROOT CAUSE]** Hybrid control architectural incompatibility

## Next Steps (Phase 1.2)

1. Run instrumented baseline tests:
   - Test MPCWBCController with torque control (should pass 30s)
   - Test WBCWalkingController in standing mode (should fail at 10s)
   - Compare diagnostic logs to identify divergence point

2. Validate hypothesis:
   - Does posture torque exceed limits in WBCWalkingController?
   - Does MPCWBCController's simpler task hierarchy avoid the issue?
   - Is hybrid control the root cause?

3. Test proposed fixes:
   - Remove unused variable (line 863)
   - Simplify task hierarchy to match MPCWBCController
   - Test with full torque control (no hybrid mode)

## References

- `src/mpc_wbc_controller.py` - Lines 1-260 (initialization, update, torque control)
- `src/wbc_walking_controller.py` - Lines 107-422, 542-694, 780-868 (params, tasks, torques, update)
- `WBC_DIAG_NOTES.md` - Test results #8, #9, #10
- `WBC_ARCHITECTURAL_REDESIGN.md` - Phase 1.1 plan

---

**Status**: Phase 1.1 Complete ✅
**Next**: Phase 1.2 - Run instrumented baseline tests

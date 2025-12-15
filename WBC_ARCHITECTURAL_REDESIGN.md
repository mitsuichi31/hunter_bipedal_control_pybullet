# Hunter WBC-Hybrid Control Integration: Architectural Redesign Plan

**Document Status**: Updated with Phase 1 Findings
**Date**: 2025-11-24 (Updated: 2025-11-24)
**Version**: 2.0

---

## 🚨 CRITICAL UPDATE: Phase 1 Investigation Complete

**Phase 1 Testing (2025-11-24)** revealed a fundamental flaw in the original analysis:

### Phase 1 Key Findings

✅ **Both MPCWBCController and WBCWalkingController FAIL with torque/hybrid control**
- MPCWBCController with torque control: Falls at t=0.12s, posture_tau=107 Nm (5.4x limit)
- WBCWalkingController with hybrid control: Falls at t=0.03s, posture_tau=109 Nm (5.5x limit)
- **Identical failure mode**: Posture torque explosion → saturation → robot falls

✅ **Torque control is NOT viable for bipedal standing** (tested empirically)
- Torque limit (20 Nm) insufficient for bipedal balance control
- Posture PD gains (Kp=15.0) cause positive feedback when saturated
- Free-floating base dynamics require stiff position control

✅ **MPCWBCController ONLY works with POSITION_CONTROL** (not torque/hybrid)
- `standing-mpc` mode uses PyBullet POSITION_CONTROL on ALL joints
- Proven stable: 30+ seconds standing (Roll=0.2°, Pitch=0.1°)
- WBC framework is NOT used for torque control in the working baseline

### Revised Strategy

**Original Plan (INCORRECT)**:
- Match WBCWalkingController architecture to MPCWBCController's working torque/hybrid control
- Problem: MPCWBCController doesn't actually work with torque/hybrid control!

**Revised Plan (CORRECT)**:
1. ✅ **Use POSITION_CONTROL** (proven stable in both controllers)
2. ✅ **Simplify task hierarchy** (2 tasks vs 4 tasks) to match MPCWBCController
3. ✅ **Remove torque/hybrid control** (empirically proven to fail)
4. ✅ **Focus on gait planning + IK** for walking mode (not WBC torque control)

**Success Probability (REVISED)**: 95% for standing mode with position control

**See**:
- `BASELINE_TEST_RESULTS.md` - Phase 1.2 empirical test data
- `CONTROLLER_COMPARISON.md` - Phase 1.1 code analysis

---

## 🎉 FINAL UPDATE: Phase 4 Complete (2025-11-26)

**Phase 4 (Position Control Walking)** successfully validated the revised strategy identified in Phase 3 investigation:

### Phase 4 Implementation Results

✅ **Alternative Approach: Pure Position Control + CoM Planning**
- Implemented `PositionControlWalkingController` (position control only, no WBC torque control)
- `SimpleCoMPlanner2D` for ZMP-based CoM trajectory planning
- `FullBodyIKSolver` for whole-body IK (base + joint angles)
- **Completely bypassed WBC torque control issues**

✅ **Achievements (6 Sessions, Nov 25-26, 2025)**:
- **Session 1**: CoM planner + Full-body IK implementation
- **Session 2-3**: Gait integration (4cm steps, 2s period)
- **Session 4**: Disturbance rejection (50-100N pushes, 100% success)
- **Session 5**: Multi-step validation (3/4 levels passed, 60s walking)
- **Session 6**: Robustness testing (5 min walk, ±20% mass, 4/4 tests passed)

### Final Performance Metrics

| Metric | Original Goal | Phase 4 Achieved | Exceeded By |
|--------|---------------|------------------|-------------|
| Walking Duration | 60s | **300s (5 min)** | **5x** |
| Consecutive Steps | 10 | **150 steps** | **15x** |
| Roll Stability | < 5° | **< 0.3°** | **16x better** |
| Pitch Stability | < 5° | **< 3.1°** | **1.6x better** |
| Disturbance Rejection | 5N | **100N** | **20x** |
| Mass Uncertainty | ±10% | **±20%** | **2x** |
| Forward Distance | - | **2.746m (5 min)** | - |

### Key Validation Points

1. **Position Control Works**: Pure position control (no torque/hybrid) achieved all walking objectives
2. **WBC Bypass Successful**: Avoided WBC-hybrid incompatibility entirely by using different architecture
3. **Long-Term Stability**: 5 minute continuous walk with minimal drift (pitch: +0.042°/min)
4. **Mass Robustness**: ±20% mass variation handled without retuning
5. **Strong Disturbance Rejection**: 100N lateral push survived (20x original goal)

### Architectural Vindication

**Phase 3 Investigation Findings (2025-11-24):**
- ✅ Identified WBC-hybrid control as fundamentally incompatible
- ✅ Recommended position control as alternative
- ✅ Predicted 95% success probability for position control approach

**Phase 4 Results (2025-11-26):**
- ✅ Position control approach succeeded beyond expectations
- ✅ All robustness tests passed (4/4)
- ✅ System production-ready for conservative gaits

### Conclusion

**The Phase 3 investigation correctly identified the root cause** (WBC assumes full actuation, hybrid control underactuates) **and the alternative approach (position control) was fully validated in Phase 4.**

The Hunter bipedal robot now demonstrates:
- ✅ **5+ minute continuous walking** without falling
- ✅ **Exceptional stability** (roll < 0.3°, pitch < 3.1°)
- ✅ **Strong disturbance rejection** (100N forces)
- ✅ **Mass robustness** (±20% uncertainty)

**Status**: Investigation complete, alternative solution implemented and validated.

**See**:
- `PHASE_4_SESSION_1-6_SUMMARY.md` - Complete Phase 4 implementation details
- `PHASE_4_WALKING_PLAN.md` - Original Phase 4 plan
- `src/position_control_walking.py` - Production implementation

---

## Executive Summary (Original - Superseded by Phase 1)

<details>
<summary>Click to expand original analysis (OUTDATED)</summary>

After comprehensive code analysis, the fundamental architectural incompatibility between the WBC framework and hybrid control mode has been identified. **WBC computes dynamics for a 10-DOF system while hybrid control only actuates 2-DOF (ankles)**, creating an underactuated system where posture errors accumulate exponentially, leading to torque saturation and failure.

**Key Finding**: The controllers differ not just in parameters, but in their fundamental control architecture. MPCWBCController has implicit architectural patterns that make hybrid control work, while WBCWalkingController explicitly violates the hybrid control constraints through its task formulation and dynamics computation.

**Recommended Solution**: **Approach B - Unified Control Architecture** (merge working patterns from MPCWBCController)

**Expected Timeline**: 5-7 days for Phase 1-2, with immediate validation possible

**Success Probability**: 85% for standing mode, 50% for walking mode

**UPDATE**: This analysis was based on incorrect assumption that MPCWBCController works with hybrid control. Phase 1 testing proves both controllers fail with torque/hybrid control.

</details>

---

## 1. Root Cause Analysis

### 1.1 Why MPCWBCController Works (30+ sec stability)

**Architecture**:
```python
# mpc_wbc_controller.py (lines 111-234)
MPCWBCController.update():
    1. Compute desired base accelerations (6D: 3 linear + 3 angular)
    2. Add height regulation to vertical acceleration
    3. WBC computes ground forces from base accelerations
    4. Hybrid mode splits: position on 8 joints, torque on 2 ankles
    5. Torques computed from contact forces via Jacobians (2 DOF only)
```

**Critical Success Factors**:

1. **Task Hierarchy Simplicity** (lines 157-183):
   - Only 2 tasks: orientation (Priority 1) + CoM tracking (Priority 1)
   - No explicit stance foot constraints
   - No swing foot tracking tasks
   - Tasks produce 6D base accelerations only

2. **Foot Anchoring in QP Objective** (lines 198-211):
   - Cartesian stiffness applied in WBC QP objective (w=10, kp=300, kd=100)
   - NOT as hard constraints, but as soft objectives
   - Allows QP solver flexibility while stabilizing feet

3. **Hybrid Control Implementation** (lines 335-392):
   - Position control: Sets target angles directly via PyBullet POSITION_CONTROL
   - Torque control: Computes full dynamics, extracts ankle torques only
   - Clear separation: position joints never receive WBC torques

4. **Inverse Dynamics Usage** (lines 394-462):
   - Computes torques for ALL 10 joints: τ_full = τ_gravity + τ_contact + τ_posture + τ_damping
   - But only extracts ankle torques (leg_l5, leg_r5) for actual application
   - Position-controlled joints governed by PyBullet's internal PD controller

### 1.2 Why WBCWalkingController Fails (10s failure)

**Architecture**:
```python
# wbc_walking_controller.py (lines 780-868)
WBCWalkingController.update():
    1. Build task hierarchy with 4 tasks (stance, orientation, CoM, swing)
    2. Get desired 6D base acceleration from tasks
    3. WBC computes ground forces
    4. Compute torques via inverse dynamics (10 DOF)
    5. Hybrid mode: extract ankle torques, apply position to hips/knees
```

**Critical Failure Mechanisms**:

1. **Explicit Stance Foot Constraints** (lines 347-366, commented out in Option B):
   - Originally had Priority 0 hard constraints on stance foot velocity
   - Overconstrains QP solver when combined with foot anchoring
   - Creates conflicting objectives in WBC optimization

2. **Complex Task Hierarchy** (lines 333-413):
   - 4 tasks: stance constraints + orientation + CoM + swing foot
   - Tasks are formulated assuming 10-DOF actuation
   - Swing foot tracking task expects joint motion that position control prevents

3. **Posture PD Conflict** (lines 542-551, 638-648):
   - Computes posture torques for all 10 joints
   - Position-controlled joints receive conflicting commands:
     - WBC dynamics: "Apply X N⋅m to achieve base stability"
     - Position control: "Hold joint at fixed angle"
   - System becomes uncontrollable as errors accumulate

4. **Diagnostic Evidence** (Test #9, #10 in WBC_DIAG_NOTES.md):
   ```
   t=0.013s: posture_norm=93.505 Nm (should be ~5 Nm)
   t=0.014s: posture_norm=109.110 Nm (4-5x torque limit!)
   force_norms=[47.08, 51.715] -> [0.006, 0.007] (500Hz oscillations)
   ```
   - Posture error explodes because position control fights WBC dynamics
   - Forces oscillate wildly as QP tries to compensate
   - Torques saturate, base becomes unstable → fall at t=10s

### 1.3 The Fundamental Problem

**WBC Framework Assumption**:
- Free-floating base dynamics: τ = M(q)q̈ + g(q)
- Mass matrix M is 10x10 (all joints actuated)
- Inverse dynamics maps base accelerations → joint torques for ALL 10 joints
- System is fully actuated (10 inputs, 10 DOF)

**Hybrid Control Reality**:
- Only 2 ankle joints apply WBC torques
- 8 hip/knee joints use position control (fixed angles)
- System is underactuated (2 inputs, 10 DOF)
- Position control introduces kinematic constraints that WBC doesn't know about

**Resulting Conflict**:
```
WBC: "To stabilize base, need τ_hip = 15 Nm, τ_knee = 20 Nm, τ_ankle = 5 Nm"
Position Control (hip/knee): "Ignoring your torque, holding fixed angle"
Ankle Torques Alone: "Can't stabilize base with only 5 Nm at ankles!"
Base Tilts → Posture Error ↑ → WBC Demands More → Saturation → FALL
```

---

## 2. Proposed Solutions (Comparative Analysis)

### Approach A: Reduced-DOF WBC ⚠️

**Concept**: Modify WBC to only compute forces/torques for actuated joints (2 ankles)

**Pros**:
- Theoretically correct for underactuated system
- Eliminates conflict between WBC and position control
- Clean mathematical formulation

**Cons**:
- ❌ **Very High Complexity**: Requires deep WBC reformulation
- ❌ **May Not Be Feasible**: 2 DOF insufficient to control 6D base motion
- ❌ **No Guarantee of Success**: Even with correct formulation, physics may not allow stability
- ❌ **Long Implementation Time**: 10-15 days estimated
- ❌ **High Risk**: Fundamental physics limits may make this impossible

**Verdict**: Not recommended. Too risky, too complex, and physically questionable.

---

### Approach B: Unified Control Architecture ✅ **RECOMMENDED**

**Concept**: Merge working architectural patterns from MPCWBCController into WBCWalkingController

**Implementation Strategy**:

**Phase 1: Task Hierarchy Simplification** (2 days)
- Reduce from 4 tasks to 2 tasks (orientation + CoM)
- Remove explicit stance foot constraints
- Keep foot anchoring in QP objective only

**Phase 2: Dynamics Computation Alignment** (2 days)
- Match height regulation implementation
- Align torque computation order
- Synchronize Jacobian usage

**Phase 3: Parameter Synchronization** (1 day)
- Match all control gains (kp/kd)
- Remove posture scaling (always 1.0)
- Align damping parameters

**Pros**:
- ✅ **Proven to Work**: MPCWBCController achieves 30+ sec stability
- ✅ **Low Risk**: Copying existing working patterns
- ✅ **Moderate Complexity**: Mostly refactoring, not new algorithms
- ✅ **Quick Validation**: Can test incrementally
- ✅ **Preserves WBC Framework**: No fundamental changes to dynamics
- ✅ **Clear Implementation Path**: Line-by-line correspondence

**Cons**:
- ⚠️ **Doesn't Address Root Cause**: Still has WBC-hybrid incompatibility
- ⚠️ **May Not Enable Walking**: Works for standing, but walking adds motion

**Verdict**: **RECOMMENDED**. Highest probability of success, lowest risk, proven approach.

---

### Approach C: Hierarchical Split ⚠️

**Concept**: Separate high-level planning (WBC) from low-level execution (position control)

**Pros**:
- ✅ **Philosophically Clean**: Separates concerns clearly
- ✅ **Leverages Position Control**: Uses robust PyBullet POSITION_CONTROL
- ✅ **Avoids Torque Conflicts**: No hybrid control needed

**Cons**:
- ❌ **Requires New IK Solver**: Map 6D base accel → 10D joint positions
- ❌ **IK May Not Exist**: Underactuated system, no unique solution
- ❌ **Long Development Time**: 8-10 days for IK solver
- ❌ **Unproven Approach**: No existing reference implementation

**Verdict**: Interesting but too risky. Requires novel IK formulation.

---

### Approach D: Constrained WBC

**Concept**: Modify WBC to treat position-controlled joints as kinematic constraints

**Pros**:
- ✅ **Mathematically Rigorous**: Proper constrained optimization
- ✅ **Respects Hybrid Control**: Explicitly models constraints
- ✅ **Preserves WBC Framework**: Extension, not replacement

**Cons**:
- ❌ **Very High Complexity**: Requires constrained QP formulation
- ❌ **Reduced Jacobian Computation**: Non-trivial with PyBullet
- ❌ **May Be Infeasible**: Constraints may make QP unsolvable
- ❌ **Long Development Time**: 12-15 days

**Verdict**: Most elegant solution, but too complex for immediate success.

---

## 3. Solution Comparison Table

| Approach | Complexity | Risk | Timeline | Success Probability | Walking Potential |
|----------|-----------|------|----------|-------------------|------------------|
| **A: Reduced-DOF WBC** | Very High | Very High | 10-15 days | 20% | Unknown |
| **B: Unified Architecture** ✅ | Medium | Low | 5-7 days | 85% | Medium (50%) |
| **C: Hierarchical Split** | High | High | 8-10 days | 40% | High (70%) |
| **D: Constrained WBC** | Very High | Medium | 12-15 days | 60% | High (80%) |

---

## 4. Recommended Approach: Detailed Implementation Plan

### Phase 1: Investigation and Validation (1-2 days)

**Objective**: Confirm architectural differences and establish baseline

**Tasks**:

1.1. **Side-by-Side Code Comparison** (4 hours)
- Compare key methods line-by-line
- Document specific differences with line numbers
- Create architectural flow diagrams

1.2. **Instrumented Test Runs** (4 hours)
- Run MPCWBCController (baseline - should pass 30s)
- Run WBCWalkingController (should fail at 10s)
- Compare logs to identify divergence point

1.3. **Create Architectural Diagram** (2 hours)
- Document control flow for both controllers
- Highlight differences in task formulation
- Map out data flow from state → commands

**Validation Criteria**:
- [ ] Confirmed MPCWBCController runs 30+ sec
- [ ] Confirmed WBCWalkingController fails at ~10s
- [ ] Identified 5+ specific architectural differences
- [ ] Created comparison document with line number references

---

### Phase 2: Core Architectural Changes (3-4 days)

**Objective**: Align WBCWalkingController architecture with MPCWBCController

#### 2.1. Task Hierarchy Simplification (Day 1)

**File**: `src/wbc_walking_controller.py` (lines 333-413)

**Changes**:
1. Simplify `_build_task_hierarchy()` to only 2 tasks:
   - Task 1: Body orientation (Priority 1) - kp=100.0, kd=3.0
   - Task 2: CoM tracking (Priority 1) - kp=50.0, kd=5.0
2. Remove explicit stance foot constraints (lines 347-366)
3. Remove swing foot tasks (incompatible with standing mode)

**Testing**:
```bash
WALKING_WBC=1 WBC_WALKING_STANDING=1 WBC_HYBRID_CONTROL=1 \
WBC_ANCHOR_WEIGHT=10 WBC_ANCHOR_KP=300 WBC_ANCHOR_KD=100 \
python3 src/main_simulation.py --mode walking --duration 10 --no-gui
```

**Expected**: Should improve stability (may not reach 30s yet)

#### 2.2. Dynamics Computation Alignment (Day 2)

**File**: `src/wbc_walking_controller.py` (lines 591-695)

**Changes**:
1. Add height regulation to base acceleration (match MPCWBCController:189-195)
2. Align torque computation order:
   - gravity_torques
   - posture_torques
   - contact_torques (via Jacobian transpose)
   - damping
3. Match diagnostic logging format

**Testing**:
```bash
WALKING_WBC=1 WBC_WALKING_STANDING=1 WBC_HYBRID_CONTROL=1 \
WBC_ANCHOR_WEIGHT=10 WBC_ANCHOR_KP=300 WBC_ANCHOR_KD=100 \
python3 src/main_simulation.py --mode walking --duration 15 --no-gui
```

**Expected**: Further improvement, possibly reaching 15-20s

#### 2.3. Parameter Synchronization (Day 3)

**File**: `src/wbc_walking_controller.py` (lines 107-159, 638-648)

**Changes**:
1. Update WBCWalkingParams defaults:
   - kp_orientation: 60.0 → 100.0
   - kd_orientation: 8.0 → 3.0
   - kp_com: 20.0 → 50.0
   - kd_com: 4.0 → 5.0
   - height_kp: → 60.0
   - height_kd: → 6.0
   - posture_kp: → 15.0
   - posture_kd: → 1.5
   - joint_damping_gain: → 0.3
2. Remove posture scaling (always 1.0)

**Testing**:
```bash
WALKING_WBC=1 WBC_WALKING_STANDING=1 WBC_HYBRID_CONTROL=1 \
WBC_ANCHOR_WEIGHT=10 WBC_ANCHOR_KP=300 WBC_ANCHOR_KD=100 \
python3 src/main_simulation.py --mode walking --duration 30 --no-gui
```

**Expected**: 30+ second stability (matching MPCWBCController)

#### 2.4. Validation Testing (Day 4)

**Test Suite**:
```bash
# Test 1: Baseline - MPCWBCController (should pass)
WBC_HYBRID_CONTROL=1 WBC_ANCHOR_WEIGHT=10 WBC_ANCHOR_KP=300 WBC_ANCHOR_KD=100 \
python3 src/main_simulation.py --mode wbc --duration 30 --no-gui

# Test 2: Updated WBCWalkingController (should now pass!)
WALKING_WBC=1 WBC_WALKING_STANDING=1 WBC_HYBRID_CONTROL=1 \
WBC_ANCHOR_WEIGHT=10 WBC_ANCHOR_KP=300 WBC_ANCHOR_KD=100 \
python3 src/main_simulation.py --mode walking --duration 30 --no-gui

# Test 3: Extended duration
WALKING_WBC=1 WBC_WALKING_STANDING=1 WBC_HYBRID_CONTROL=1 \
WBC_ANCHOR_WEIGHT=10 WBC_ANCHOR_KP=300 WBC_ANCHOR_KD=100 \
python3 src/main_simulation.py --mode walking --duration 60 --no-gui
```

**Success Criteria**:
- [ ] WBCWalkingController standing: 30+ sec stable
- [ ] Roll < 1°, Pitch < 1°
- [ ] Height stable (~0.689m ± 0.01m)
- [ ] Forces balanced (each foot ~62 N)
- [ ] No torque saturation (<20 Nm)
- [ ] Matches MPCWBCController performance

---

### Phase 3: Integration and Testing (1-2 days)

**Objective**: Ensure robustness and document changes

#### 3.1. Comprehensive Testing (Day 1)

**Test Categories**:

1. **Baseline Comparison**:
   - Run both controllers side-by-side
   - Compare stability metrics
   - Verify performance parity

2. **Stress Tests**:
   - Long duration (120s)
   - Parameter variations
   - Robustness validation

3. **Regression Tests**:
   - Ensure other modes still work
   - Validate no breaking changes

#### 3.2. Documentation (Day 2)

**Files to Update**:

1. **WBC_DIAG_NOTES.md**: Add Test #11 results
2. **STABILITY_IMPROVEMENT_PLAN.md**: Mark Phase 3.1 complete
3. **ARCHITECTURAL_ALIGNMENT.md** (new): Document alignment process

---

### Phase 4: Extension to Walking (Future)

**Objective**: Enable actual walking motion (not part of immediate plan)

**Approach** (for future implementation):

1. **Gait State Machine**: Add controlled CoM shifting
2. **Swing Foot Task (Conditional)**: Only during actual swing phase
3. **Hybrid Control Extension**: Consider switching swing leg to torque control

**Estimated Timeline**: 10-15 days after Phase 3 completion

---

## 5. Key Architectural Differences

| Aspect | MPCWBCController | WBCWalkingController | Impact |
|--------|-----------------|---------------------|--------|
| **Task Count** | 2 (orientation + CoM) | 4 (stance + orientation + CoM + swing) | Complex QP |
| **Stance Constraints** | None (foot anchoring only) | Explicit Priority 0 constraints | Overconstraining |
| **Height Regulation** | Added to base accel (lines 189-195) | Missing or incorrect | Base drift |
| **Posture Scaling** | Always 1.0 | Variable 0.1-0.25 in standing | Weak posture hold |
| **Control Gains** | kp_orientation=100, kd=3 | kp_orientation=60, kd=8 | Response mismatch |
| **CoM Gains** | kp_com=50, kd=5 | kp_com=20, kd=4 | Tracking error |
| **Damping** | 0.3 always | 0.1-0.3 variable | Oscillations |

---

## 6. Parameter Changes Required

| Parameter | Old Value | New Value | Source |
|-----------|-----------|-----------|--------|
| kp_orientation | 60.0 | 100.0 | MPCWBCController:168 |
| kd_orientation | 8.0 | 3.0 | MPCWBCController:169 |
| kp_com | 20.0 | 50.0 | MPCWBCController:180 |
| kd_com | 4.0 | 5.0 | MPCWBCController:181 |
| height_kp | (varied) | 60.0 | MPCWBCController:93 |
| height_kd | (varied) | 6.0 | MPCWBCController:94 |
| posture_kp | 8.0-15.0 | 15.0 | MPCWBCController:89 |
| posture_kd | 0.8-1.5 | 1.5 | MPCWBCController:90 |
| joint_damping_gain | 0.1-0.3 | 0.3 | MPCWBCController:91 |
| diag_posture_scale | 0.1-0.25 | (removed) | Always 1.0 |

---

## 7. Success Criteria

### Phase 2 Success Criteria (Core Implementation)

**After Phase 2.1 (Task Hierarchy)**:
- [ ] Robot does not crash before t=10s
- [ ] Roll/Pitch improved from baseline (Test #10)
- [ ] No immediate posture explosion

**After Phase 2.2 (Dynamics Alignment)**:
- [ ] Robot stable for 15+ seconds
- [ ] Roll < 2°, Pitch < 2°
- [ ] Forces show reduced oscillations

**After Phase 2.3 (Parameter Sync)**:
- [ ] Robot stable for 30+ seconds
- [ ] Roll < 1°, Pitch < 1°
- [ ] Height stable (~0.689m ± 0.01m)
- [ ] Forces balanced (60-65 N per foot)
- [ ] Torques < 15 Nm average, < 20 Nm max
- [ ] Matches MPCWBCController performance within 10%

### Phase 3 Success Criteria (Validation)

**Stability Metrics**:
- [ ] 30-second test: 100% success rate (5/5 runs)
- [ ] 60-second test: 80% success rate (4/5 runs)
- [ ] Roll: < 1° (matches MPCWBCController)
- [ ] Pitch: < 1° (matches MPCWBCController)
- [ ] Height: 0.689m ± 0.01m

**Force Metrics**:
- [ ] Total GRF = robot weight ± 5N
- [ ] Left/right balance: within 10 N
- [ ] Force oscillations < 5 N peak-to-peak
- [ ] ZMP error from center < 5 cm

**Torque Metrics**:
- [ ] Average torque < 10 Nm per joint
- [ ] Max torque < 20 Nm (no saturation)
- [ ] Posture norm < 50 Nm (was 87-109 Nm in failed tests)

---

## 8. Risk Assessment and Mitigation

### High-Priority Risks

**Risk 1: Architecture Alignment Insufficient**
- **Probability**: Medium (30%)
- **Impact**: High (blocks entire approach)
- **Mitigation**:
  - Incremental testing after each phase
  - Side-by-side comparison with MPCWBCController
  - Early detection via 10s/15s intermediate tests
  - Fallback: Try Approach C (Hierarchical Split)

**Risk 2: Hidden Architectural Differences**
- **Probability**: Medium (40%)
- **Impact**: Medium (requires debugging)
- **Mitigation**:
  - Detailed code comparison in Phase 1
  - Instrumented logging to identify divergence
  - Line-by-line verification of key methods

**Risk 3: Parameter Sensitivity**
- **Probability**: Low (20%)
- **Impact**: Low (solvable via tuning)
- **Mitigation**:
  - Start with exact parameter match
  - Test parameter variations systematically
  - Document sensitivity ranges

---

## 9. Conclusion and Recommendations

### Summary

**Problem**: WBCWalkingController fails at t=10s with posture torque explosion (87-109 Nm) while MPCWBCController achieves 30+ sec stability, despite identical environment variables.

**Root Cause**: Architectural incompatibility between WBC framework (assumes 10-DOF actuation) and hybrid control (only 2-DOF ankle actuation). Complex task hierarchy with explicit stance constraints amplifies the conflict.

**Recommended Solution**: **Approach B - Unified Control Architecture**

**Implementation**: Align WBCWalkingController with MPCWBCController by:
1. Simplifying task hierarchy (2 tasks vs 4)
2. Removing explicit stance foot constraints
3. Matching dynamics computation exactly
4. Synchronizing all control parameters

**Timeline**: 5-7 days for standing stability, 15-20 days total for walking extension

**Success Probability**: 85% for standing mode, 50% for walking mode

### Why This Approach Will Work

1. **Proven Pattern**: MPCWBCController already achieves 30+ sec stability with the same hybrid control configuration

2. **Architectural Simplicity**: Reducing task complexity eliminates QP solver conflicts and constraint satisfaction issues

3. **Parameter Alignment**: Exact parameter matching ensures identical control behavior

4. **Incremental Validation**: Each phase has testable success criteria, allowing early detection of issues

5. **Low Risk**: Copying working patterns is safer than inventing new algorithms

### Next Steps

**Immediate Actions**:
1. Review and approve this architectural redesign plan
2. Create development branch: `git checkout -b wbc-architectural-alignment`
3. Start Phase 1: Investigation and validation (1-2 days)
4. Set up automated testing framework
5. Document baseline metrics for comparison

**First Milestone** (End of Week 1):
- Phase 1 & Phase 2.1 complete
- Task hierarchy simplified
- Initial stability improvement demonstrated (10+ seconds)

**Second Milestone** (End of Week 2):
- Phase 2.2 & 2.3 complete
- Full architectural alignment achieved
- 30+ second standing stability validated

---

## Appendix: Line Number References

**MPCWBCController (Working)**:
- Lines 111-234: update() main control loop
- Lines 157-183: Task hierarchy (2 tasks only)
- Lines 189-195: Height regulation
- Lines 198-211: Foot anchoring in QP
- Lines 335-392: Hybrid control implementation
- Lines 394-462: Torque computation

**WBCWalkingController (Failing)**:
- Lines 780-868: update() main control loop
- Lines 333-413: Task hierarchy (4 tasks, complex)
- Lines 347-366: Explicit stance constraints
- Lines 591-695: Torque computation
- Lines 638-648: Posture scaling (problematic)

**Key Files**:
- `src/mpc_wbc_controller.py` (640 lines) - Working reference
- `src/wbc_walking_controller.py` (914 lines) - Needs alignment
- `src/wbc_controller.py` (368 lines) - Core WBC QP solver
- `src/inverse_dynamics.py` (431 lines) - Dynamics computation
- `WBC_DIAG_NOTES.md` (516 lines) - Diagnostic history

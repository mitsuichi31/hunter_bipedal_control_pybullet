# Phase 4: Robust Dynamic Walking via Position Control + CoM Planning

**Date**: 2025-11-25
**Status**: 📋 PLANNING
**Approach**: Novel pure position control architecture (avoids torque control entirely)

---

## Executive Summary

**Goal**: Achieve indefinite bipedal walking with disturbance rejection using position control

**Key Innovation**: Combine proven position control stability (Phase 2) with intelligent CoM trajectory planning to solve IK's fixed-base limitation (Phase 3)

**Architecture**:
```
Gait Planner → Foot Trajectories + CoM Trajectory
      ↓
CoM Controller → Desired Base Position/Velocity (ZMP-aware weight shift)
      ↓
Full-Body IK → Joint Angles (base motion + foot tracking)
      ↓
Position Control → PyBullet POSITION_CONTROL (proven stable)
      ↓
Feedback Loop → State estimation → Disturbance rejection
```

**Estimated Timeline**: 2-3 weeks (6 phases)

**Success Criteria**:
- ✅ 10+ consecutive steps without falling
- ✅ Walking speed: 0.05-0.15 m/s (slow to moderate)
- ✅ Disturbance rejection: Recovers from 5N lateral push
- ✅ Indefinite walking: 60+ seconds continuous
- ✅ Stability: Roll < 5°, Pitch < 5° during walking

---

## Why This Approach Will Work

### 1. Leverages Proven Stability (Phase 2)
- Position control achieved Roll=0.0°, Pitch=-1.8° for standing
- No torque saturation issues
- PyBullet's internal PD controller is robust and well-tuned

### 2. Solves IK's Fixed-Base Problem (Phase 3)
- Phase 3 failed because PyBullet IK assumes fixed base
- **Solution**: Explicitly compute base trajectory from CoM planning
- Feed desired base position directly to full-body IK solver

### 3. Avoids Torque Control Issues (Phase 1)
- Torque control failed with 20 Nm limit (required 107-125 Nm)
- Position control has no torque limit (PyBullet handles internally)
- More robust to model uncertainties

### 4. Builds on Existing Infrastructure
- ✅ `GaitGenerator`: Foot trajectory planning (already implemented)
- ✅ `BipedalIKSolver`: Leg IK (already implemented)
- ✅ `stability_metrics.py`: CoM/ZMP calculation (Phase 1)
- ✅ Contact state machine (already working)
- 🆕 Need: CoM trajectory planner
- 🆕 Need: Full-body IK (base + legs)

---

## Phase 4 Breakdown

### Phase 4.1: CoM Trajectory Planner (Week 1, Days 1-3)

**Objective**: Design ZMP-based CoM trajectory generator for dynamic balance

**Implementation Options**:

#### Option A: Preview Control (Kajita's Method) ⭐ **RECOMMENDED**
```python
class PreviewCoMPlanner:
    """
    Preview control for ZMP-based CoM trajectory generation
    Based on Kajita et al. (2003) - "Biped Walking Pattern Generation"

    Key idea: Plan CoM trajectory that produces desired ZMP trajectory
    """
    def __init__(self, preview_steps=100):
        # Linear inverted pendulum model: ẍ = (g/h) * (x - p)
        # where p is ZMP position
        self.com_height = 0.689  # meters (from Phase 2)
        self.preview_horizon = 1.0  # seconds
        self.dt = 0.01  # 100 Hz planning

    def plan_com_trajectory(self, desired_zmp, current_com, current_vel):
        """
        Compute CoM trajectory that tracks desired ZMP

        Input:
            desired_zmp: [N x 2] array of (x, y) ZMP positions over time
            current_com: [2] current CoM position
            current_vel: [2] current CoM velocity

        Output:
            com_trajectory: [N x 2] planned CoM positions
            com_velocity: [N x 2] planned CoM velocities
        """
        # Use preview control to compute optimal CoM trajectory
        # This ensures ZMP stays inside support polygon
        pass
```

**Pros**:
- Well-established method (used in Honda ASIMO, HRP-2)
- Mathematically rigorous (optimal control)
- Handles ZMP constraints explicitly
- Preview horizon allows smooth anticipation

**Cons**:
- Moderate complexity (need to understand preview control)
- Requires tuning preview gain

**Implementation Time**: 2-3 days

#### Option B: MPC-Based CoM Planning
```python
class MPCCoMPlanner:
    """
    Model Predictive Control for CoM trajectory
    Reuse existing MPC infrastructure from mpc_controller.py
    """
    def __init__(self):
        self.horizon = 10  # steps
        # Reuse MPC from mpc_controller.py
        # Extend to include ZMP constraints

    def plan_com_trajectory(self, foot_contacts, desired_zmp):
        # Optimize CoM trajectory subject to:
        # 1. ZMP inside support polygon
        # 2. Smooth acceleration
        # 3. Track desired ZMP from gait planner
        pass
```

**Pros**:
- Leverage existing MPC code (`mpc_controller.py`)
- Flexible constraint handling
- Can optimize multiple objectives

**Cons**:
- Current MPC assumes standing (no swing phase)
- Need to extend for dynamic walking
- Computational overhead

**Implementation Time**: 3-4 days

#### Option C: Simple ZMP Heuristic
```python
class SimpleCoMPlanner:
    """
    Heuristic CoM planning based on foot position
    Simpler but less optimal than preview control
    """
    def plan_com_trajectory(self, support_foot_pos, swing_foot_pos, phase):
        # During double support: CoM between feet
        # During single support: CoM over stance foot with offset

        if phase == "double_support":
            com_target = (support_left + support_right) / 2
        else:  # single support
            com_target = support_foot_pos + [0.0, 0.0]  # Directly above

        # Add smooth transition between phases
        return smooth_interpolate(current_com, com_target, dt)
```

**Pros**:
- Very simple to implement (1 day)
- Easy to understand and debug
- Fast computation

**Cons**:
- Not optimal (may be inefficient)
- No explicit ZMP tracking
- Limited disturbance rejection

**Implementation Time**: 1 day

**Recommendation**: Start with **Option A (Preview Control)** - it's the industry standard and provides the best foundation for disturbance rejection.

---

### Phase 4.2: Full-Body IK Solver (Week 1, Days 4-5)

**Objective**: Implement IK that solves for base position + leg joints simultaneously

**Current Limitation** (Phase 3):
```python
# Phase 3 approach (WRONG for walking)
left_target_base = left_target - base_pos  # Assumes base_pos is fixed!
ik_solution = ik_solver.solve_left_leg(left_target_base)
```

**New Approach**:
```python
class FullBodyIKSolver:
    """
    Solve for base position + joint angles simultaneously
    Constraints:
    1. Left foot at target position (if in contact)
    2. Right foot at target position (if in contact)
    3. CoM at planned position (from CoM planner)
    4. Base orientation upright (roll/pitch near zero)
    """

    def solve(self, left_foot_target, right_foot_target, com_target,
              left_contact, right_contact):
        """
        Solve constrained optimization:

        minimize: ||q - q_current||^2  (stay close to current config)

        subject to:
            FK_left(q) = left_foot_target   (if left_contact)
            FK_right(q) = right_foot_target  (if right_contact)
            CoM(q) = com_target
            roll ≈ 0, pitch ≈ 0
            joint_limits

        where q = [base_pos, base_orn, joint_angles]

        Returns:
            base_pos: [3] optimized base position
            base_orn: [4] optimized base orientation (quaternion)
            joint_angles: [10] optimized joint configuration
        """
        pass
```

**Implementation Options**:

#### Option A: Nonlinear Optimization (scipy) ⭐ **RECOMMENDED**
```python
from scipy.optimize import minimize

def full_body_ik_objective(q, targets, contacts, weights):
    """
    q = [base_x, base_y, base_z, base_roll, base_pitch, base_yaw,
         joint1, ..., joint10]
    """
    # Compute forward kinematics for current q
    left_foot_pos = FK_left(q)
    right_foot_pos = FK_right(q)
    com_pos = CoM(q)

    # Cost terms
    cost = 0.0
    if contacts[0]:  # left foot in contact
        cost += weights['foot'] * ||left_foot_pos - targets['left_foot']||^2
    if contacts[1]:  # right foot in contact
        cost += weights['foot'] * ||right_foot_pos - targets['right_foot']||^2

    cost += weights['com'] * ||com_pos - targets['com']||^2
    cost += weights['orientation'] * (q[3]^2 + q[4]^2)  # Penalize roll/pitch
    cost += weights['regularization'] * ||q - q_prev||^2

    return cost

# Solve
result = minimize(full_body_ik_objective, q_init, method='SLSQP',
                  bounds=joint_limits, constraints=contact_constraints)
```

**Pros**:
- Handles all constraints simultaneously
- Flexible objective function
- Proven optimization library

**Cons**:
- May be slow (need to tune solver settings)
- Requires good initialization (use current state)

**Implementation Time**: 2 days

#### Option B: Iterative PyBullet IK
```python
def iterative_full_body_ik(targets, contacts, max_iter=10):
    """
    Alternate between:
    1. Adjust base position to match CoM target
    2. Solve leg IK with updated base position
    3. Repeat until converged
    """
    base_pos = current_base_pos
    for i in range(max_iter):
        # Step 1: Leg IK with current base
        joint_angles = solve_leg_ik(targets, base_pos)

        # Step 2: Compute resulting CoM
        actual_com = compute_com(base_pos, joint_angles)

        # Step 3: Adjust base to correct CoM error
        base_pos += k_com * (target_com - actual_com)

        if ||target_com - actual_com|| < tolerance:
            break

    return base_pos, joint_angles
```

**Pros**:
- Leverages existing PyBullet IK
- Simple to implement
- Fast iteration

**Cons**:
- May not converge for all configurations
- Heuristic approach (not mathematically optimal)

**Implementation Time**: 1 day

**Recommendation**: **Option A (Nonlinear Optimization)** - more robust and handles all constraints properly.

---

### Phase 4.3: Gait Integration (Week 2, Days 1-2)

**Objective**: Integrate gait planner + CoM planner + full-body IK

**Architecture**:
```python
class PositionControlWalkingController:
    """
    Pure position control walking controller
    No torque control, no WBC - just intelligent position commands
    """

    def __init__(self, robot_id, config):
        self.gait_generator = GaitGenerator(config.gait)  # Existing
        self.com_planner = PreviewCoMPlanner(config.com_planning)  # Phase 4.1
        self.full_body_ik = FullBodyIKSolver(robot_id)  # Phase 4.2

        # State
        self.phase = "double_support"
        self.step_count = 0

    def update(self, robot_state, dt):
        # 1. Get foot targets from gait generator
        gait_phase = self.gait_generator.update(dt)
        left_foot_target = gait_phase['left_foot']
        right_foot_target = gait_phase['right_foot']
        contact_state = gait_phase['contacts']

        # 2. Compute desired ZMP trajectory
        desired_zmp = self.compute_desired_zmp(contact_state,
                                                left_foot_target,
                                                right_foot_target)

        # 3. Plan CoM trajectory to track ZMP
        com_trajectory = self.com_planner.plan_com_trajectory(
            desired_zmp=desired_zmp,
            current_com=robot_state['com'],
            current_vel=robot_state['com_vel']
        )
        com_target = com_trajectory[0]  # First step of trajectory

        # 4. Solve full-body IK
        ik_solution = self.full_body_ik.solve(
            left_foot_target=left_foot_target,
            right_foot_target=right_foot_target,
            com_target=com_target,
            left_contact=contact_state[0],
            right_contact=contact_state[1]
        )

        # 5. Extract position commands
        joint_positions = ik_solution['joint_angles']

        # 6. Apply position control
        return {joint: {'mode': 'position', 'value': pos}
                for joint, pos in joint_positions.items()}

    def compute_desired_zmp(self, contacts, left_target, right_target):
        """
        Compute desired ZMP based on contact state

        Double support: ZMP between feet (interpolate based on phase)
        Single support: ZMP at stance foot
        """
        if contacts == (True, True):  # Double support
            # Interpolate ZMP from left to right (or vice versa)
            # based on gait phase
            phase_ratio = self.gait_generator.get_phase_ratio()
            zmp = left_target * (1 - phase_ratio) + right_target * phase_ratio
        elif contacts == (True, False):  # Left stance
            zmp = left_target
        elif contacts == (False, True):  # Right stance
            zmp = right_target
        else:
            # No contact? Emergency - use current CoM projection
            zmp = robot_state['com'][:2]  # x, y only

        return zmp
```

**Integration Steps**:
1. Create new file: `src/position_control_walking.py`
2. Integrate with existing `main_simulation.py`
3. Add new mode: `--mode walking-position`
4. Test with very conservative gait parameters first

**Conservative Gait Parameters** (for initial testing):
```yaml
gait:
  step_length: 0.02      # 2cm steps (very small)
  step_height: 0.01      # 1cm lift (minimal)
  step_period: 2.0       # 2 seconds per step (very slow)
  double_support_ratio: 0.6  # 60% double support (stable)
```

**Implementation Time**: 2 days

---

### Phase 4.4: Disturbance Rejection (Week 2, Days 3-4)

**Objective**: Add feedback control for robustness

**Key Components**:

#### 4.4.1: State Estimation
```python
class StateEstimator:
    """
    Estimate robot state for feedback control
    """
    def __init__(self):
        self.com_filter = LowPassFilter(cutoff=10.0)  # 10 Hz
        self.velocity_estimator = VelocityEstimator()

    def estimate(self, robot_state):
        # Filter noisy measurements
        com_filtered = self.com_filter.update(robot_state['com'])
        com_vel = self.velocity_estimator.update(com_filtered)

        # Estimate base acceleration (for ZMP)
        com_accel = self.velocity_estimator.get_acceleration()

        # Compute actual ZMP
        zmp_actual = compute_zmp(com_filtered, com_accel, height=0.689)

        return {
            'com': com_filtered,
            'com_vel': com_vel,
            'com_accel': com_accel,
            'zmp': zmp_actual,
            'base_orn': robot_state['base_orn']
        }
```

#### 4.4.2: Feedback Control
```python
class FeedbackController:
    """
    Correct CoM plan based on actual state
    """
    def __init__(self):
        self.zmp_kp = 0.5  # ZMP error feedback gain
        self.orientation_kp = 0.3  # Orientation error feedback gain

    def compute_correction(self, desired_state, actual_state):
        # ZMP error feedback
        zmp_error = desired_state['zmp'] - actual_state['zmp']
        com_correction_zmp = self.zmp_kp * zmp_error

        # Orientation error feedback
        euler_actual = p.getEulerFromQuaternion(actual_state['base_orn'])
        roll_error = -euler_actual[0]  # Want roll = 0
        pitch_error = -euler_actual[1]  # Want pitch = 0

        # Translate orientation error to CoM correction
        # (leaning forward → shift CoM forward to compensate)
        com_correction_orientation = [
            self.orientation_kp * pitch_error,
            self.orientation_kp * roll_error
        ]

        # Total correction
        total_correction = com_correction_zmp + com_correction_orientation

        return total_correction

# In main control loop:
def update_with_feedback(self, robot_state, dt):
    # 1-3. [Same as Phase 4.3 - compute base plan]

    # 4. State estimation
    estimated_state = self.state_estimator.estimate(robot_state)

    # 5. Compute feedback correction
    desired_state = {'zmp': desired_zmp, ...}
    correction = self.feedback_controller.compute_correction(
        desired_state, estimated_state
    )

    # 6. Apply correction to CoM target
    com_target_corrected = com_target + correction

    # 7. [Continue with IK as before]
```

**Disturbance Scenarios to Test**:
1. **Lateral push**: Apply 5N force to robot body for 0.1s
2. **Ground unevenness**: Add random terrain height variations (±5mm)
3. **Model uncertainty**: Increase robot mass by 10%
4. **Sensor noise**: Add Gaussian noise to state measurements

**Implementation Time**: 2 days

---

### Phase 4.5: Multi-Step Walking Validation (Week 3, Days 1-2)

**Objective**: Validate walking performance with increasing complexity

**Test Progression**:

#### Level 1: Minimal Walking (Day 1 morning)
```yaml
Parameters:
  step_length: 0.02 m
  step_height: 0.01 m
  step_period: 2.0 s
  double_support: 0.6

Success Criteria:
  - 3 consecutive steps
  - No falls
  - Roll/Pitch < 10°
```

#### Level 2: Slow Walking (Day 1 afternoon)
```yaml
Parameters:
  step_length: 0.05 m
  step_height: 0.02 m
  step_period: 1.5 s
  double_support: 0.5

Success Criteria:
  - 10 consecutive steps
  - No falls
  - Roll/Pitch < 8°
  - Walking speed ≈ 0.033 m/s
```

#### Level 3: Moderate Walking (Day 2 morning)
```yaml
Parameters:
  step_length: 0.10 m
  step_height: 0.03 m
  step_period: 1.0 s
  double_support: 0.4

Success Criteria:
  - 20 consecutive steps
  - No falls
  - Roll/Pitch < 5°
  - Walking speed ≈ 0.10 m/s
```

#### Level 4: Indefinite Walking (Day 2 afternoon)
```yaml
Parameters:
  step_length: 0.10 m
  step_height: 0.03 m
  step_period: 1.0 s
  double_support: 0.4

Success Criteria:
  - 60+ seconds continuous walking
  - 50+ consecutive steps
  - No falls
  - Roll/Pitch < 5°
  - Stable gait pattern
```

**Test Commands**:
```bash
# Level 1
WALKING_POS_CONTROL=1 GAIT_STEP_LENGTH=0.02 GAIT_STEP_PERIOD=2.0 \
  python3 src/main_simulation.py --mode walking-position --duration 10

# Level 2
WALKING_POS_CONTROL=1 GAIT_STEP_LENGTH=0.05 GAIT_STEP_PERIOD=1.5 \
  python3 src/main_simulation.py --mode walking-position --duration 20

# Level 3
WALKING_POS_CONTROL=1 GAIT_STEP_LENGTH=0.10 GAIT_STEP_PERIOD=1.0 \
  python3 src/main_simulation.py --mode walking-position --duration 30

# Level 4
WALKING_POS_CONTROL=1 GAIT_STEP_LENGTH=0.10 GAIT_STEP_PERIOD=1.0 \
  python3 src/main_simulation.py --mode walking-position --duration 60
```

**Implementation Time**: 2 days (includes debugging and parameter tuning)

---

### Phase 4.6: Robustness Testing (Week 3, Days 3-5)

**Objective**: Validate disturbance rejection and robustness

**Test Suite**:

#### Test 1: Lateral Push Disturbance
```python
def apply_lateral_push(sim, time_to_push=5.0):
    """
    Apply 5N lateral force to robot body at specified time
    """
    if abs(sim.time - time_to_push) < 0.01:
        p.applyExternalForce(
            objectUniqueId=sim.robot_id,
            linkIndex=-1,  # Base
            forceObj=[0, 5.0, 0],  # 5N in Y direction
            posObj=[0, 0, 0],
            flags=p.LINK_FRAME
        )

# Test command
WALKING_POS_CONTROL=1 DISTURBANCE_TEST=lateral_push \
  python3 src/main_simulation.py --mode walking-position --duration 15
```

**Success Criteria**:
- Robot continues walking after push
- Recovers within 2-3 steps
- Does not fall

#### Test 2: Uneven Terrain
```python
def create_uneven_terrain(sim):
    """
    Create terrain with random height variations
    """
    # Add random bumps (±5mm) to ground plane
    pass

# Test command
WALKING_POS_CONTROL=1 TERRAIN=uneven \
  python3 src/main_simulation.py --mode walking-position --duration 30
```

**Success Criteria**:
- Completes 30s walking on uneven terrain
- Adapts foot placement to terrain
- Maintains stability (Roll/Pitch < 8°)

#### Test 3: Model Uncertainty
```bash
# Increase robot mass by 10%
WALKING_POS_CONTROL=1 MASS_SCALE=1.1 \
  python3 src/main_simulation.py --mode walking-position --duration 30
```

**Success Criteria**:
- Walking remains stable with model mismatch
- No retuning required
- Demonstrates robustness to parameter variations

#### Test 4: Continuous Walking Stress Test
```bash
# 5 minute continuous walking
WALKING_POS_CONTROL=1 \
  python3 src/main_simulation.py --mode walking-position --duration 300
```

**Success Criteria**:
- 300 seconds (5 minutes) continuous walking
- No performance degradation over time
- Stable gait pattern maintained

**Implementation Time**: 3 days (includes creating test scenarios and analysis)

---

## Timeline Summary

| Phase | Task | Duration | Week |
|-------|------|----------|------|
| **4.1** | CoM Trajectory Planner | 3 days | Week 1 |
| **4.2** | Full-Body IK Solver | 2 days | Week 1 |
| **4.3** | Gait Integration | 2 days | Week 2 |
| **4.4** | Disturbance Rejection | 2 days | Week 2 |
| **4.5** | Multi-Step Validation | 2 days | Week 3 |
| **4.6** | Robustness Testing | 3 days | Week 3 |
| | **Buffer / Documentation** | 2 days | Week 3 |
| | **TOTAL** | **16 days** | **~3 weeks** |

---

## Success Criteria (Final Validation)

### Minimum Success (Phase 4.5 Level 2)
- ✅ 10+ consecutive steps
- ✅ No falls during 20-second test
- ✅ Roll < 8°, Pitch < 8°
- ✅ Walking speed: 0.03-0.05 m/s

### Target Success (Phase 4.5 Level 4)
- ✅ 60+ seconds continuous walking
- ✅ 50+ consecutive steps
- ✅ Roll < 5°, Pitch < 5°
- ✅ Walking speed: 0.08-0.12 m/s
- ✅ Stable gait pattern (repeatable step timing)

### Stretch Success (Phase 4.6 Complete)
- ✅ All minimum + target criteria
- ✅ Recovers from 5N lateral push
- ✅ Walks on uneven terrain (±5mm variations)
- ✅ Robust to 10% mass uncertainty
- ✅ 5+ minutes continuous walking
- ✅ Disturbance rejection demonstrated

---

## Risk Analysis

### High-Priority Risks

#### Risk 1: Full-Body IK Convergence
**Problem**: Optimization may not converge for all configurations
**Probability**: Medium (40%)
**Impact**: High (blocks entire approach)
**Mitigation**:
- Start with good initialization (current state)
- Use regularization to stay close to current config
- Fallback: Iterative PyBullet IK approach (Option B)
- Test IK solver in isolation before integration

#### Risk 2: CoM Planning Instability
**Problem**: Preview control may produce unstable trajectories
**Probability**: Medium (30%)
**Impact**: Medium (requires retuning)
**Mitigation**:
- Start with conservative gait parameters
- Validate CoM planner in standing mode first
- Use ZMP safety margin (stay 2cm inside support polygon)
- Fallback: Simple heuristic planner (Option C)

#### Risk 3: Position Control Lag
**Problem**: PyBullet position control may be too slow for dynamic walking
**Probability**: Low (20%)
**Impact**: Medium (need to slow down gait)
**Mitigation**:
- Increase gait period if needed (slower walking)
- Use high position control gains
- Reduce step length/height
- Validate with Phase 4.5 Level 1 first

#### Risk 4: Contact Transition Instability
**Problem**: Foot strikes may cause instability (impact forces)
**Probability**: Medium (35%)
**Impact**: Medium (requires tuning)
**Mitigation**:
- Use smooth foot trajectories (quintic polynomials)
- Increase double support ratio initially
- Add compliance in foot contact (PyBullet contact settings)
- Gradual transition in CoM planner

### Medium-Priority Risks

#### Risk 5: Computational Performance
**Problem**: Optimization may be too slow for real-time control
**Probability**: Low (15%)
**Impact**: Low (can reduce control frequency)
**Mitigation**:
- Use compiled optimization (scipy with BLAS)
- Warm-start optimization with previous solution
- Run at lower control frequency if needed (50 Hz instead of 1 kHz)

---

## Alternative Fallback Plans

### If Phase 4.2 (Full-Body IK) Fails

**Fallback Option**: Decoupled IK + Base Adjustment
```python
def simplified_approach(foot_targets, com_target):
    # 1. Estimate required base position from CoM target
    base_pos = estimate_base_from_com(com_target)

    # 2. Solve leg IK with estimated base position
    joint_angles = solve_leg_ik(foot_targets, base_pos)

    # 3. Return position commands (no base control)
    return joint_angles
```

**Trade-off**: Less accurate CoM tracking, but simpler

### If Phase 4.1 (CoM Planning) is Too Complex

**Fallback Option**: Static Stability Margins
```python
def conservative_com_planning(support_polygon):
    # Always keep CoM 5cm inside support polygon
    # Use simple geometric center of support polygon
    com_target = compute_support_center(support_polygon) + safety_margin
    return com_target
```

**Trade-off**: Slower walking, but guaranteed stability

---

## Key Design Decisions

### Decision 1: CoM Planning Method
- **Options**: Preview Control, MPC, Simple Heuristic
- **Recommendation**: Preview Control (industry standard)
- **Rationale**: Best balance of optimality and complexity
- **Review Point**: End of Phase 4.1

### Decision 2: IK Solver Approach
- **Options**: Nonlinear optimization, Iterative PyBullet IK
- **Recommendation**: Nonlinear optimization
- **Rationale**: Handles all constraints simultaneously
- **Review Point**: End of Phase 4.2

### Decision 3: Feedback Control Strategy
- **Options**: ZMP feedback, Orientation feedback, Combined
- **Recommendation**: Combined (ZMP + orientation)
- **Rationale**: Addresses both balance and posture errors
- **Review Point**: End of Phase 4.4

### Decision 4: Gait Parameters
- **Initial**: Very conservative (2cm steps, 2s period)
- **Target**: Moderate (10cm steps, 1s period)
- **Stretch**: Aggressive (15cm steps, 0.8s period)
- **Review Point**: Throughout Phase 4.5

---

## Expected Outcomes

### Best Case (80% probability)
- All phases complete successfully
- Achieve stretch success criteria
- 5+ minutes continuous walking
- Robust disturbance rejection
- **Timeline**: 16 days (on schedule)

### Nominal Case (90% probability)
- All phases complete with minor issues
- Achieve target success criteria
- 60+ seconds continuous walking
- Basic disturbance rejection
- **Timeline**: 18-20 days (1 week buffer)

### Worst Case (95% probability)
- Phase 4.2 or 4.1 requires fallback
- Achieve minimum success criteria
- 10+ consecutive steps
- Limited disturbance rejection
- **Timeline**: 21-25 days (2 weeks buffer)

### Failure Scenario (<5% probability)
- Fundamental limitation discovered (e.g., position control too slow)
- Cannot achieve stable walking
- **Fallback**: Document findings, propose Phase 5 alternative

---

## Documentation Plan

### During Development
- Daily progress log (commit messages)
- Phase completion summaries (like Phase 1-3)
- Test results with metrics

### Final Documentation
1. **PHASE_4_WALKING_SUMMARY.md**
   - Complete technical description
   - Test results and analysis
   - Lessons learned

2. **POSITION_CONTROL_WALKING_GUIDE.md**
   - User guide for running walking mode
   - Parameter tuning guide
   - Troubleshooting

3. **Update CLAUDE.md**
   - Add Phase 4 architecture
   - Update status and achievements
   - Add walking mode commands

4. **Create FINAL_PROJECT_SUMMARY.md**
   - Complete project overview
   - All phases documented
   - Future work recommendations

---

## Next Steps (After Plan Approval)

### Step 1: Review and Approve Plan
- Review this document
- Discuss design decisions
- Approve approach or request modifications

### Step 2: Create Development Branch
```bash
git checkout -b phase4-position-control-walking
```

### Step 3: Start Phase 4.1
- Implement preview control CoM planner
- Test in isolation (no robot, just trajectory generation)
- Validate ZMP tracking

### Step 4: Incremental Progress
- Complete each phase sequentially
- Test at each milestone
- Document results

---

## Questions for Review

1. **CoM Planning Method**: Preview Control vs MPC vs Simple Heuristic?
   - Recommendation: Preview Control
   - Your preference: ?

2. **IK Solver Approach**: Nonlinear optimization vs Iterative PyBullet?
   - Recommendation: Nonlinear optimization
   - Your preference: ?

3. **Timeline**: 3 weeks acceptable?
   - Minimum success: 2 weeks
   - Target success: 3 weeks
   - Stretch success: 4 weeks
   - Your timeline: ?

4. **Success Criteria**: Which level is required?
   - Minimum: 10 steps
   - Target: 60s walking
   - Stretch: 5min + disturbances
   - Your requirement: ?

5. **Validation Priority**: What matters most?
   - Walking duration
   - Walking speed
   - Disturbance rejection
   - Gait smoothness
   - Your priority: ?

---

**Status**: 📋 Awaiting review and approval

**Next Action**: Review this plan, provide feedback, approve or request changes

# Hunter Bipedal Robot - Stability Improvement Plan

**Date**: November 23, 2025
**Status**: Planning Phase
**Goal**: Enhance stability control and enable robust walking

---

## Executive Summary

The Hunter bipedal robot currently achieves **excellent standing stability** (Roll=0.2°, Pitch=0.1°) using passive straight-leg configuration with minimal active control. However, **advanced control modes (WBC, Walking) require architectural improvements** to achieve their full potential.

This document outlines a **phased development plan** to:
1. ✅ Improve core stability fundamentals (CoM, ZMP, gravity compensation)
2. 🔧 Tune and validate WBC control for dynamic stability
3. 🚶 Enable robust bipedal walking with contact-aware control
4. 📊 Add comprehensive stability monitoring and diagnostics

---

## Current Status Assessment

### What Works ✅

| Component | Status | Performance |
|-----------|--------|-------------|
| **Standing Mode (PD)** | ✅ Excellent | Roll=0.2°, Pitch=0.1°, Height=0.691m |
| **MPC Balance Mode** | ✅ Good | Active CoM stabilization functional |
| **Joint Control** | ✅ Good | Tunable PD gains per joint |
| **Simulation** | ✅ Excellent | Stable 1kHz PyBullet integration |
| **URDF/Meshes** | ✅ Fixed | Both legs render correctly |

### What Needs Improvement ⚠️

| Component | Status | Issues |
|-----------|--------|--------|
| **WBC Control** | ⚠️ Partial | QP solver needs parameter tuning |
| **CoM Calculation** | ⚠️ Simplified | Uses base link only, should use all links |
| **ZMP Computation** | ⚠️ Approximate | No force sensors, simplified calculation |
| **Walking Mode** | ❌ Broken | IK incompatible with free-floating base |
| **Inverse Dynamics** | ❌ Missing | No mass matrix/Coriolis computation |
| **Gravity Compensation** | ❌ Missing | No feedforward torque compensation |

---

## Development Phases

### Phase 1: Core Stability Fundamentals (Week 1-2)
**Goal**: Improve fundamental stability computations

#### 1.1 Accurate Center of Mass Calculation
**Priority**: 🔴 Critical
**Effort**: 2-3 days
**File**: `src/stability_metrics.py` (new)

**Current Issue**:
```python
# balance_controller.py:85 - WRONG
base_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
return base_pos  # Only base link!
```

**Solution**:
```python
def compute_com(robot_id):
    """Compute true CoM from all links"""
    total_mass = 0
    com_pos = np.zeros(3)

    # Base link
    base_mass, base_pos = get_link_mass_and_pos(robot_id, -1)
    total_mass += base_mass
    com_pos += base_mass * base_pos

    # All other links
    for i in range(p.getNumJoints(robot_id)):
        link_mass, link_pos = get_link_mass_and_pos(robot_id, i)
        total_mass += link_mass
        com_pos += link_mass * link_pos

    return com_pos / total_mass
```

**Expected Impact**: ±2-5cm CoM accuracy improvement, better MPC predictions

#### 1.2 True ZMP Computation with Dynamics
**Priority**: 🔴 Critical
**Effort**: 3-4 days
**File**: `src/stability_metrics.py`

**Current Issue**:
```python
# balance_controller.py:120 - Oversimplified
zmp = com_pos[0:2]  # Just CoM projection
```

**Solution** (with simulated force sensors):
```python
def compute_zmp_with_dynamics(robot_id):
    """Compute ZMP using acceleration and inertia"""
    com_pos, com_vel, com_acc = get_com_state_full(robot_id)

    # ZMP formula: zmp_x = x - (h/g) * ddot_x
    h = com_pos[2]  # Height above ground
    g = 9.81

    zmp_x = com_pos[0] - (h / g) * com_acc[0]
    zmp_y = com_pos[1] - (h / g) * com_acc[1]

    return np.array([zmp_x, zmp_y])
```

**Expected Impact**: Predictive stability (know if falling before it happens)

#### 1.3 Gravity Compensation
**Priority**: 🟡 High
**Effort**: 3-4 days
**File**: `src/gravity_compensation.py` (new)

**Solution**:
```python
def compute_gravity_torques(robot_id, joint_positions):
    """Compute gravity compensation torques"""
    # Use PyBullet's calculateMassMatrix
    mass_matrix = p.calculateMassMatrix(robot_id, joint_positions)

    # Gravity vector in joint space
    gravity_torques = compute_gravity_vector(robot_id, joint_positions)

    return gravity_torques
```

**Integration**:
```python
# In pd_controller.py
control_torque = pd_torque + gravity_compensation_torque
```

**Expected Impact**: 20-30% torque efficiency improvement, smoother control

**Phase 1 Deliverables**:
- ✅ `src/stability_metrics.py` - CoM, ZMP, stability margin computation
- ✅ `src/gravity_compensation.py` - Joint-space gravity compensation
- ✅ Updated `balance_controller.py` - Use accurate CoM/ZMP
- ✅ Updated `pd_controller.py` - Add gravity feedforward
- 📊 Test results: Standing mode with metrics visualization

---

### Phase 2: WBC Tuning & Validation (Week 3-4)
**Goal**: Make Whole-Body Control mode functional and stable

#### 2.1 QP Parameter Tuning
**Priority**: 🔴 Critical
**Effort**: 4-5 days
**File**: `src/wbc_controller.py`, `src/mpc_wbc_controller.py`

**Current Issue**: QP occasionally infeasible, robot falls

**Parameters to Tune**:
```python
# wbc_controller.py
w_force_tracking = 1.0      # Track desired GRF distribution
w_force_regularization = 0.1  # Minimize force magnitude
w_cop_tracking = 10.0       # Track ZMP/CoP target
friction_coeff = 0.5        # Ground friction

# Task hierarchy weights
w_com_tracking = 100.0      # CoM position error
w_orientation = 50.0        # Body orientation
w_joint_regularization = 0.01  # Joint motion smoothness
```

**Systematic Tuning Process**:
1. Start with conservative weights (force regularization high)
2. Gradually increase force tracking weight
3. Add orientation control incrementally
4. Validate with standing tests
5. Add disturbance rejection tests

**Expected Impact**: WBC mode achieves standing stability comparable to PD mode

#### 2.2 Implement Inverse Dynamics
**Priority**: 🟡 High
**Effort**: 5-6 days
**File**: `src/inverse_dynamics.py` (new)

**Why Needed**: WBC computes desired forces, need to map to joint torques

**Implementation**:
```python
class InverseDynamics:
    def compute_mass_matrix(self, q):
        """M(q): Joint-space mass matrix"""
        return p.calculateMassMatrix(self.robot_id, q)

    def compute_coriolis(self, q, qd):
        """C(q,qd): Coriolis/centrifugal forces"""
        # Recursive Newton-Euler algorithm
        return self._compute_coriolis_recursive(q, qd)

    def compute_gravity(self, q):
        """g(q): Gravity torques"""
        return self._compute_gravity_torques(q)

    def inverse_dynamics(self, q, qd, qdd_desired):
        """Compute torques for desired acceleration"""
        M = self.compute_mass_matrix(q)
        C = self.compute_coriolis(q, qd)
        g = self.compute_gravity(q)

        # τ = M(q)*qdd + C(q,qd)*qd + g(q)
        return M @ qdd_desired + C @ qd + g
```

**Expected Impact**: Accurate torque control, proper WBC force distribution

#### 2.3 Contact Force Validation
**Priority**: 🟡 High
**Effort**: 2-3 days
**File**: `src/wbc_controller.py`

**Validation Tests**:
1. **Static standing**: GRF should equal body weight
2. **Lean forward**: More force on toes, less on heels
3. **Lean sideways**: Asymmetric left/right distribution
4. **Push disturbance**: Forces should react to maintain balance

**Metrics to Track**:
```python
# Total vertical force should equal weight
total_fz = left_foot_fz + right_foot_fz
weight = robot_mass * 9.81
assert abs(total_fz - weight) < 5.0  # Within 5N

# ZMP should be inside support polygon
zmp = compute_zmp_from_forces(left_force, right_force)
assert is_inside_support_polygon(zmp, foot_positions)
```

**Phase 2 Deliverables**:
- ✅ Tuned WBC parameters (standing mode functional)
- ✅ `src/inverse_dynamics.py` - Dynamics computation library
- ✅ Force validation tests passing
- ✅ WBC standing mode: Roll<1°, Pitch<1°
- 📊 Force distribution analysis plots

---

### Phase 3: Walking Mode Redesign (Week 5-7)
**Goal**: Enable robust bipedal walking

#### 3.1 WBC-Based Walking Architecture
**Priority**: 🔴 Critical
**Effort**: 6-8 days
**File**: `src/wbc_walking_controller.py` (redesign)

**Current Architecture (BROKEN)**:
```
Gait Generator → IK Solver → PD Controller
Problem: IK assumes fixed base (violates free-floating dynamics)
```

**New Architecture (WBC-BASED)**:
```
Gait Generator → Contact Planner → WBC → Inverse Dynamics → Torques
Benefits: Handles free-floating base, contact transitions, momentum
```

**Key Components**:

1. **Contact State Machine**:
```python
class ContactState:
    DOUBLE_SUPPORT = 0  # Both feet on ground
    LEFT_SWING = 1      # Right foot stance, left swinging
    RIGHT_SWING = 2     # Left foot stance, right swinging
    FLIGHT = 3          # Both feet off ground (avoid!)
```

2. **WBC Task Hierarchy for Walking**:
```python
# Priority 1: Contact constraints (must satisfy)
- Stance foot fixed (0 velocity)
- Friction cone constraints
- Unilateral force constraints (no pull)

# Priority 2: Motion objectives (optimize)
- CoM trajectory tracking (from MPC)
- Swing foot trajectory tracking (from gait generator)
- Body orientation control (upright)

# Priority 3: Regularization (minimize)
- Joint velocity limits
- Smooth force distribution
- Energy efficiency
```

3. **Contact Transition Handling**:
```python
def handle_contact_transition(phase):
    if phase == HEEL_STRIKE:
        # Add new contact constraint smoothly
        contact_jacobian = update_contacts(new_foot="left")
        increase_contact_weight(from=0.0, to=1.0, duration=0.05)

    elif phase == TOE_OFF:
        # Remove contact constraint smoothly
        decrease_contact_weight(from=1.0, to=0.0, duration=0.05)
        remove_contact(foot="right")
```

**Expected Impact**: Stable walking without "flying" issues

#### 3.2 Gait Parameter Optimization
**Priority**: 🟡 High
**Effort**: 3-4 days
**File**: `src/gait_generator.py`

**Current Parameters** (conservative):
```python
step_length = 0.1      # meters
step_height = 0.05     # meters
step_period = 1.0      # seconds (very slow!)
stance_width = 0.18    # meters
```

**Optimization Process**:
1. Start with slow gait (step_period = 1.5s)
2. Validate stability with metrics
3. Gradually reduce period (1.5s → 1.2s → 1.0s → 0.8s)
4. Tune step height for energy efficiency
5. Optimize step length for speed

**Target Parameters**:
```python
step_length = 0.15     # meters (50% increase)
step_height = 0.04     # meters (20% decrease - more efficient)
step_period = 0.8      # seconds (25% faster)
```

**Expected Impact**: 0.19 m/s walking speed (from 0.10 m/s)

#### 3.3 Momentum-Based Control
**Priority**: 🟠 Medium
**Effort**: 4-5 days
**File**: `src/momentum_control.py` (new)

**Why Needed**: Dynamic walking requires angular momentum management

**Implementation**:
```python
class MomentumController:
    def compute_centroidal_momentum(self, robot_id):
        """Linear + angular momentum at CoM"""
        # h = [linear_momentum, angular_momentum]
        # h = Σ(m_i * v_i, I_i * ω_i + r_i × m_i * v_i)
        return momentum_6d

    def momentum_rate_control(self, h_desired, h_current):
        """Control momentum rate (hdot)"""
        # hdot = Σ(external_forces)
        # Use as constraint/objective in WBC
        return momentum_rate_command
```

**Integration with WBC**:
```python
# Add momentum tracking task
tasks.append({
    'type': 'momentum_tracking',
    'weight': 10.0,
    'target': desired_momentum_rate
})
```

**Expected Impact**: Smoother gait transitions, natural walking dynamics

**Phase 3 Deliverables**:
- ✅ Redesigned `wbc_walking_controller.py` with contact-aware WBC
- ✅ `src/momentum_control.py` - Centroidal momentum tracking
- ✅ Optimized gait parameters
- ✅ Walking mode: 5+ stable steps without falling
- 📊 Walking stability metrics (ZMP, CoM trajectory)
- 🎥 Demonstration video

---

### Phase 4: Monitoring & Robustness (Week 8)
**Goal**: Add diagnostics and improve robustness

#### 4.1 Real-Time Stability Monitoring
**Priority**: 🟡 High
**Effort**: 2-3 days
**File**: `src/stability_monitor.py` (new)

**Metrics Dashboard**:
```python
class StabilityMonitor:
    def compute_stability_margin(self, zmp, support_polygon):
        """Distance from ZMP to polygon edge"""
        margin = distance_to_polygon_edge(zmp, support_polygon)
        return margin

    def predict_fall_time(self, com_state, zmp):
        """Time until instability if no correction"""
        # Use LIPM dynamics to predict
        return time_to_instability

    def get_metrics(self):
        return {
            'com_error': norm(com_desired - com_actual),
            'zmp_error': norm(zmp_desired - zmp_actual),
            'stability_margin': margin,
            'orientation_error': [roll_error, pitch_error],
            'joint_tracking_error': rms_joint_error,
            'fall_risk': 'LOW' | 'MEDIUM' | 'HIGH'
        }
```

**Visualization** (optional):
- Real-time plots of CoM, ZMP, support polygon
- Joint tracking error graphs
- Force distribution visualization

**Expected Impact**: Early warning system, better debugging

#### 4.2 Disturbance Rejection Tests
**Priority**: 🟡 High
**Effort**: 2 days
**File**: `tests/stability_tests.py` (new)

**Test Suite**:
```python
def test_push_disturbance():
    """Apply lateral push, verify recovery"""
    # Apply 50N push for 0.1s
    apply_external_force(robot_id, force=[50, 0, 0], duration=0.1)

    # Check recovery within 2 seconds
    assert check_stable_within(timeout=2.0)
    assert final_roll < 5.0  # degrees
    assert final_pitch < 5.0

def test_uneven_terrain():
    """One foot 2cm higher than other"""
    # Modify ground plane
    # Verify balance maintained

def test_payload():
    """Add 2kg mass to torso"""
    # Verify standing still stable
```

**Expected Impact**: Quantified robustness, confidence in deployment

#### 4.3 Parameter Auto-Tuning
**Priority**: 🟠 Medium
**Effort**: 3-4 days
**File**: `src/auto_tuner.py` (new)

**Approach**: Bayesian optimization of controller gains

```python
def tune_pd_gains(target_metrics):
    """Automatically tune PD gains for best tracking"""
    # Search space
    param_space = {
        'kp': (50, 500),
        'kd': (5, 50)
    }

    # Objective: minimize tracking error + smoothness
    def objective(kp, kd):
        error = simulate_with_gains(kp, kd)
        return error.mean() + 0.1 * error.std()

    # Bayesian optimization
    best_params = bayesian_optimize(objective, param_space)
    return best_params
```

**Expected Impact**: Optimal gains without manual tuning

**Phase 4 Deliverables**:
- ✅ `src/stability_monitor.py` - Real-time metrics
- ✅ `tests/stability_tests.py` - Robustness test suite
- ✅ `src/auto_tuner.py` - Parameter optimization (optional)
- 📊 Stability report with all tests passing
- 📄 Final performance benchmarks

---

## Implementation Priority Matrix

| Task | Impact | Effort | Priority | Phase |
|------|--------|--------|----------|-------|
| **Accurate CoM calculation** | 🔴 High | Low | P0 | 1 |
| **True ZMP computation** | 🔴 High | Medium | P0 | 1 |
| **WBC parameter tuning** | 🔴 High | Medium | P0 | 2 |
| **Inverse dynamics** | 🟡 Medium | High | P1 | 2 |
| **Gravity compensation** | 🟡 Medium | Medium | P1 | 1 |
| **WBC walking redesign** | 🔴 High | Very High | P1 | 3 |
| **Gait optimization** | 🟡 Medium | Medium | P2 | 3 |
| **Stability monitoring** | 🟡 Medium | Low | P2 | 4 |
| **Momentum control** | 🟠 Low | High | P3 | 3 |
| **Auto-tuning** | 🟠 Low | Medium | P3 | 4 |

**Priority Definitions**:
- **P0**: Critical for basic functionality
- **P1**: Important for stability/performance
- **P2**: Nice to have, improves usability
- **P3**: Future enhancement, low priority

---

## Success Criteria

### Phase 1 Success Criteria
- ✅ CoM calculation accuracy: <2cm error vs ground truth
- ✅ ZMP computation includes acceleration terms
- ✅ Gravity compensation reduces tracking error by >20%
- ✅ Standing mode maintains stability with new metrics

### Phase 2 Success Criteria
- ✅ WBC mode achieves standing: Roll<1°, Pitch<1°
- ✅ Force distribution matches expected (total = weight)
- ✅ QP solver feasible >99% of timesteps
- ✅ No robot falls in 60-second standing test

### Phase 3 Success Criteria
- ✅ Walking mode: 10+ consecutive steps without falling
- ✅ Gait speed: >0.15 m/s
- ✅ ZMP stays within support polygon >95% of time
- ✅ Smooth contact transitions (no jerking)

### Phase 4 Success Criteria
- ✅ Stability monitor provides 0.5s advance warning before fall
- ✅ Robot recovers from 50N push within 2 seconds
- ✅ All automated tests pass
- ✅ Walking mode stable on slight terrain variations

---

## Dependencies & Prerequisites

### Required Libraries
- ✅ `pybullet` - Physics simulation
- ✅ `numpy` - Numerical computation
- ✅ `cvxpy` or `qpsolvers` - QP optimization (already in WBC)
- ✅ `matplotlib` - Plotting (for monitoring)
- 🔧 `scipy` - Optimization (for auto-tuning)

### Optional Tools
- 🔧 `pinocchio` - Advanced dynamics library (alternative to custom inverse dynamics)
- 🔧 `casadi` - Nonlinear optimization (for advanced MPC)
- 🔧 `meshcat` - 3D visualization (for debugging)

### Knowledge Requirements
- Linear algebra (Jacobians, mass matrices)
- Control theory (PD, MPC, QP)
- Bipedal locomotion theory (ZMP, LIPM, contact dynamics)
- PyBullet API familiarity

---

## Risk Assessment

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| **WBC QP infeasibility** | Medium | High | Conservative tuning, fallback to PD |
| **Inverse dynamics bugs** | Medium | Medium | Use PyBullet validation, unit tests |
| **Walking instability** | High | High | Incremental testing, conservative gait |
| **Performance degradation** | Low | Low | Profile code, optimize bottlenecks |
| **Contact detection errors** | Medium | Medium | Smooth transitions, force thresholds |

---

## Timeline Estimate

**Total Duration**: 8 weeks (full-time)

| Phase | Duration | Calendar |
|-------|----------|----------|
| **Phase 1**: Core Fundamentals | 2 weeks | Week 1-2 |
| **Phase 2**: WBC Tuning | 2 weeks | Week 3-4 |
| **Phase 3**: Walking Redesign | 3 weeks | Week 5-7 |
| **Phase 4**: Monitoring & Polish | 1 week | Week 8 |

**Adjustments**:
- Part-time development: 16 weeks (4 months)
- Minimal viable product (P0 only): 4 weeks
- Full feature set (P0-P2): 8 weeks

---

## Next Steps

### Immediate Actions (This Week)
1. ✅ Review and approve this plan
2. 🔧 Set up development branch: `git checkout -b stability-improvements`
3. 🔧 Create Phase 1 task board (GitHub Issues or similar)
4. 🔧 Start with CoM calculation (easiest, high impact)

### First Milestone (End of Week 1)
- 📄 `src/stability_metrics.py` completed
- 📄 `src/gravity_compensation.py` completed
- ✅ Standing mode with accurate metrics
- 📊 Baseline metrics documented

### First Review (End of Week 2)
- Phase 1 complete
- Performance comparison: before/after
- Decision: Proceed to Phase 2 or iterate?

---

## References & Resources

### Academic Papers
- **LIPM Control**: Kajita et al. "Biped Walking Pattern Generation"
- **WBC for Humanoids**: Sentis et al. "Whole-Body Control Framework"
- **ZMP Stability**: Vukobratović et al. "Zero-Moment Point"
- **Contact-Aware WBC**: MIT Cheetah 3 papers

### Code References
- Current codebase: `/src/*.py` (analysis in this document)
- PyBullet examples: `pybullet/examples/`
- Open-source bipeds: Cassie, Atlas, Valkyrie controllers

### Tools & Documentation
- PyBullet docs: https://pybullet.org/
- CVXPY QP solver: https://www.cvxpy.org/
- NumPy linear algebra: https://numpy.org/doc/stable/reference/routines.linalg.html

---

## Conclusion

This plan provides a **structured, phased approach** to improve the Hunter robot's stability and enable walking. The foundation is solid (standing mode works excellently), and we've identified clear paths to:

1. **Improve accuracy** of fundamental computations (CoM, ZMP)
2. **Unlock WBC potential** through parameter tuning and inverse dynamics
3. **Enable walking** via contact-aware WBC redesign
4. **Ensure robustness** with monitoring and testing

**Estimated effort**: 8 weeks full-time for complete implementation.

**Recommended start**: Phase 1 (Core Fundamentals) - high impact, low risk, builds foundation for everything else.

---

**Document Status**: ✅ Ready for Review
**Next Update**: After Phase 1 completion
**Maintained By**: Development Team

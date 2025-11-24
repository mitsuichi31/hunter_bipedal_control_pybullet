# Phase 3: Walking Mode Redesign - Implementation Plan

**Date Created**: November 24, 2025
**Status**: Planning
**Goal**: Enable robust bipedal walking using WBC-based architecture
**Estimated Duration**: 6-8 days (may be 1-2 days based on Phase 1 & 2 velocity)

---

## Executive Summary

Phase 3 will replace the broken IK-based walking approach with a WBC-based architecture that properly handles free-floating base dynamics. We'll leverage all Phase 1 & 2 achievements (accurate CoM/ZMP, gravity compensation, inverse dynamics, working WBC controller).

**Key Insight from Previous Investigation**: The current walking mode fails because PyBullet's IK assumes a fixed base, causing massive foot positioning errors (50-250cm) and making the robot "fly". WBC can handle free-floating dynamics correctly.

---

## Prerequisites (Already Complete ✅)

From Phase 1 & 2, we have:
- ✅ Accurate CoM computation (`stability_metrics.py`)
- ✅ Dynamic ZMP calculation (`stability_metrics.py`)
- ✅ Gravity compensation (`gravity_compensation.py`)
- ✅ Inverse dynamics (`inverse_dynamics.py`)
- ✅ Working WBC controller (`wbc_controller.py`)
- ✅ Working MPC-WBC integration (`mpc_wbc_controller.py`)
- ✅ Existing gait generator (`gait_generator.py`)

**What's Missing**: Integration of swing foot trajectory tracking into WBC framework.

---

## Phase 3 Architecture

### Current (Broken)
```
Gait Generator → IK Solver → PD Controller
                  ↑
                  Problem: Assumes fixed base
```

### New (WBC-Based)
```
Gait Generator → WBC Task Hierarchy → QP Solver → Inverse Dynamics → Joint Torques
     ↓                   ↓
Contact State      Priority 1: Contact constraints (stance foot)
   Machine         Priority 2: Swing foot trajectory + CoM tracking
                   Priority 3: Regularization (smooth motion)
```

---

## Implementation Plan

### Milestone 1: Contact State Machine (Days 1-2)

#### Task 1.1: Implement Contact State Machine
**File**: `src/contact_state_machine.py` (NEW)
**Effort**: 4-6 hours
**Dependencies**: None

**Implementation**:
```python
class ContactPhase(Enum):
    DOUBLE_SUPPORT = 0  # Both feet on ground
    LEFT_SWING = 1      # Right stance, left swinging
    RIGHT_SWING = 2     # Left stance, right swinging

class ContactStateMachine:
    def __init__(self, step_period: float):
        self.phase = ContactPhase.DOUBLE_SUPPORT
        self.phase_time = 0.0
        self.step_period = step_period
        self.swing_ratio = 0.6  # 60% swing, 40% double support

    def update(self, dt: float) -> ContactPhase:
        """Update state machine based on time"""
        self.phase_time += dt

        # State transitions based on gait timing
        if self.phase_time >= self.step_period:
            self.phase_time = 0.0
            # Transition: DS → RS → DS → LS → DS

    def get_contact_state(self) -> Tuple[bool, bool]:
        """Returns (left_contact, right_contact)"""
        if self.phase == DOUBLE_SUPPORT:
            return (True, True)
        elif self.phase == LEFT_SWING:
            return (False, True)
        elif self.phase == RIGHT_SWING:
            return (True, False)
```

**Tests**:
- State transitions occur at correct times
- Contact flags match expected phase
- Phase timing is consistent

**Deliverable**: `src/contact_state_machine.py` with unit tests

---

#### Task 1.2: Integrate Contact Detection
**File**: `src/contact_state_machine.py`
**Effort**: 2-3 hours
**Dependencies**: Task 1.1

**Implementation**:
```python
def detect_ground_contact(self, robot_id: int, foot_link_idx: int) -> bool:
    """Detect if foot is in contact with ground"""
    # Get contact points from PyBullet
    contact_points = p.getContactPoints(
        bodyA=robot_id,
        linkIndexA=foot_link_idx
    )

    # Check vertical force threshold
    if len(contact_points) > 0:
        normal_force = contact_points[0][9]  # Normal force
        return normal_force > 5.0  # 5N threshold
    return False
```

**Tests**:
- Correctly detects foot on ground
- Correctly detects foot in air
- Handles edge cases (light contact, bouncing)

**Deliverable**: Contact detection integrated with state machine

---

### Milestone 2: WBC Walking Tasks (Days 2-4)

#### Task 2.1: Create Swing Foot Tracking Task
**File**: `src/wbc_tasks.py` (MODIFY)
**Effort**: 4-6 hours
**Dependencies**: None (extends existing)

**Implementation**:
```python
class SwingFootTask(WBCTask):
    """Track desired swing foot trajectory"""

    def __init__(self, foot_name: str, kp: float = 100.0, kd: float = 10.0):
        self.foot_name = foot_name
        self.kp = kp
        self.kd = kd

    def compute_desired_acceleration(self,
                                    current_pos: np.ndarray,
                                    desired_pos: np.ndarray,
                                    current_vel: np.ndarray,
                                    desired_vel: np.ndarray) -> np.ndarray:
        """PD control in task space"""
        pos_error = desired_pos - current_pos
        vel_error = desired_vel - current_vel

        # Desired acceleration: a_des = kp * e_pos + kd * e_vel
        return self.kp * pos_error + self.kd * vel_error
```

**Tests**:
- Task correctly computes desired acceleration
- PD gains produce stable tracking
- Integrates with existing task hierarchy

**Deliverable**: Swing foot task in `wbc_tasks.py`

---

#### Task 2.2: Create Stance Foot Constraint
**File**: `src/wbc_tasks.py` (MODIFY)
**Effort**: 3-4 hours
**Dependencies**: None

**Implementation**:
```python
class StanceFootConstraint(WBCTask):
    """Constrain stance foot to zero velocity (fixed on ground)"""

    def __init__(self, foot_name: str):
        self.foot_name = foot_name
        self.priority = 0  # Highest priority - must satisfy

    def get_constraint_jacobian(self, robot_state) -> np.ndarray:
        """Jacobian for foot velocity = 0"""
        # J_foot * qd = 0
        return self.compute_foot_jacobian(robot_state)

    def get_constraint_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Zero velocity bounds"""
        return (np.zeros(6), np.zeros(6))  # [vx, vy, vz, wx, wy, wz] = 0
```

**Tests**:
- Stance foot velocity constrained to zero
- Constraint is enforced by QP solver
- No slip during stance phase

**Deliverable**: Stance foot constraint in `wbc_tasks.py`

---

#### Task 2.3: Integrate with Gait Generator
**File**: `src/wbc_walking_controller.py` (REDESIGN)
**Effort**: 6-8 hours
**Dependencies**: Tasks 1.1, 1.2, 2.1, 2.2

**Implementation**:
```python
class WBCWalkingController:
    def __init__(self, robot_id, joint_dict, gait_params, wbc_params):
        self.robot_id = robot_id
        self.joint_dict = joint_dict

        # Components
        self.gait_generator = GaitGenerator(gait_params)
        self.contact_fsm = ContactStateMachine(gait_params.step_period)
        self.wbc = WholeBodyController(robot_id, joint_dict, wbc_params)
        self.inv_dyn = InverseDynamics(robot_id)

        # Task hierarchy
        self.task_hierarchy = TaskHierarchy()

    def update(self, dt: float) -> Dict[str, float]:
        """Main control loop"""
        # 1. Update contact state
        contact_phase = self.contact_fsm.update(dt)
        left_contact, right_contact = self.contact_fsm.get_contact_state()

        # 2. Get desired trajectories from gait generator
        left_foot_target, right_foot_target = self.gait_generator.get_foot_targets(dt)

        # 3. Build task hierarchy based on contact phase
        self.task_hierarchy.clear_tasks()

        # Priority 0: Stance foot constraints
        if left_contact:
            self.task_hierarchy.add_constraint(StanceFootConstraint("left"))
        if right_contact:
            self.task_hierarchy.add_constraint(StanceFootConstraint("right"))

        # Priority 1: Body orientation (always upright)
        self.task_hierarchy.add_task(
            create_body_orientation_task(kp=100.0, kd=10.0)
        )

        # Priority 2: CoM tracking (from MPC)
        self.task_hierarchy.add_task(
            create_com_tracking_task(kp=50.0, kd=5.0)
        )

        # Priority 3: Swing foot tracking
        if not left_contact:
            self.task_hierarchy.add_task(
                SwingFootTask("left", target=left_foot_target, kp=100.0, kd=10.0)
            )
        if not right_contact:
            self.task_hierarchy.add_task(
                SwingFootTask("right", target=right_foot_target, kp=100.0, kd=10.0)
            )

        # 4. Solve WBC QP
        desired_accelerations = self.task_hierarchy.solve()

        # 5. Compute torques via inverse dynamics
        torques = self.inv_dyn.inverse_dynamics(
            joint_positions=self.get_joint_positions(),
            joint_velocities=self.get_joint_velocities(),
            desired_accelerations=desired_accelerations
        )

        return torques
```

**Tests**:
- Integration test with all components
- Tasks are prioritized correctly
- Controller updates at correct frequency

**Deliverable**: Complete `wbc_walking_controller.py` redesign

---

### Milestone 3: Contact Transition Handling (Days 4-5)

#### Task 3.1: Implement Smooth Contact Transitions
**File**: `src/wbc_walking_controller.py`
**Effort**: 4-6 hours
**Dependencies**: Task 2.3

**Implementation**:
```python
class ContactTransitionManager:
    def __init__(self, transition_duration: float = 0.05):
        self.transition_duration = transition_duration  # 50ms
        self.in_transition = False
        self.transition_time = 0.0

    def handle_heel_strike(self, foot_name: str):
        """Smoothly add new contact constraint"""
        # Gradually increase contact weight from 0 to 1
        alpha = min(1.0, self.transition_time / self.transition_duration)
        contact_weight = alpha

        # Add contact constraint with interpolated weight
        self.add_contact_constraint(foot_name, weight=contact_weight)

    def handle_toe_off(self, foot_name: str):
        """Smoothly remove contact constraint"""
        # Gradually decrease contact weight from 1 to 0
        alpha = min(1.0, self.transition_time / self.transition_duration)
        contact_weight = 1.0 - alpha

        # Remove contact constraint gradually
        self.add_contact_constraint(foot_name, weight=contact_weight)
```

**Tests**:
- Transitions complete in specified duration
- No jerking or sudden force changes
- ZMP remains stable during transition

**Deliverable**: Smooth contact transitions

---

#### Task 3.2: Add Safety Checks
**File**: `src/wbc_walking_controller.py`
**Effort**: 2-3 hours
**Dependencies**: Task 3.1

**Implementation**:
```python
def check_stability(self) -> bool:
    """Check if robot is stable, abort if not"""
    # Check 1: ZMP inside support polygon
    zmp = compute_zmp(self.robot_id)
    support_polygon = self.get_support_polygon()
    if not is_inside_polygon(zmp, support_polygon):
        return False

    # Check 2: Orientation within limits
    roll, pitch, yaw = self.get_orientation()
    if abs(roll) > 15 or abs(pitch) > 15:  # degrees
        return False

    # Check 3: CoM height reasonable
    com_height = compute_com(self.robot_id)[2]
    if com_height < 0.4 or com_height > 0.8:
        return False

    return True
```

**Tests**:
- Detects instability before falling
- Aborts walking gracefully
- Returns to safe stance

**Deliverable**: Safety checks integrated

---

### Milestone 4: Gait Tuning & Testing (Days 5-6)

#### Task 4.1: Start with Ultra-Conservative Gait
**File**: `src/wbc_walking_controller.py`
**Effort**: 2 hours
**Dependencies**: Milestone 3 complete

**Initial Parameters**:
```python
ultra_conservative_gait = {
    'step_length': 0.05,      # 5cm (very small)
    'step_height': 0.03,      # 3cm (very low)
    'step_period': 2.0,       # 2 seconds (very slow)
    'double_support_ratio': 0.5,  # 50% of cycle
    'stance_width': 0.18,     # Standard width
}
```

**Success Criteria**:
- Robot takes 1 step without falling
- ZMP stays in support polygon
- No slip on stance foot

**Deliverable**: Ultra-conservative gait working

---

#### Task 4.2: Incremental Gait Tuning
**File**: Gait parameters
**Effort**: 6-8 hours
**Dependencies**: Task 4.1

**Tuning Process**:
1. **Phase 1**: Validate 1 step (period=2.0s)
2. **Phase 2**: Increase to 3 steps (period=2.0s)
3. **Phase 3**: Increase to 10 steps (period=2.0s)
4. **Phase 4**: Speed up to 1.5s period
5. **Phase 5**: Speed up to 1.2s period
6. **Phase 6**: Increase step length to 0.08m
7. **Phase 7**: Increase step length to 0.10m
8. **Phase 8**: Final optimization

**Metrics to Track**:
- Success rate (% of trials without falling)
- ZMP margin (distance to polygon edge)
- Energy efficiency (integrated torque²)
- Walking speed (m/s)

**Target Parameters**:
```python
optimized_gait = {
    'step_length': 0.10,      # 10cm
    'step_height': 0.04,      # 4cm
    'step_period': 1.0,       # 1 second
    'double_support_ratio': 0.3,  # 30% of cycle
    'stance_width': 0.18,
}
```

**Deliverable**: Tuned gait parameters achieving Phase 3 success criteria

---

#### Task 4.3: Create Walking Test Suite
**File**: `src/test_wbc_walking.py` (NEW)
**Effort**: 3-4 hours
**Dependencies**: Task 4.2

**Test Cases**:
1. **Single Step Test**: Robot takes 1 step and stops
2. **Continuous Walking Test**: Robot walks 10+ steps
3. **Speed Test**: Measure walking speed
4. **Stability Test**: ZMP stays in polygon >95% of time
5. **Robustness Test**: Small external perturbations

**Deliverable**: Comprehensive test suite

---

### Milestone 5: Integration & Polish (Days 6-7)

#### Task 5.1: Integrate with main_simulation.py
**File**: `src/main_simulation.py`
**Effort**: 2-3 hours
**Dependencies**: Milestone 4 complete

**Implementation**:
```python
def run_walking_test(duration: float = 10.0, use_gui: bool = True):
    """Test robot with WBC walking controller"""
    # Initialize simulation
    sim = HunterSimulation(...)

    # Create WBC walking controller
    gait_params = GaitParams(
        step_length=0.10,
        step_period=1.0,
        ...
    )

    wbc_params = WBCParams(
        friction_coef=0.6,
        w_force_tracking=10.0,
        ...
    )

    controller = WBCWalkingController(
        robot_id=sim.robot_id,
        joint_dict=sim.joint_dict,
        gait_params=gait_params,
        wbc_params=wbc_params
    )

    # Control loop
    while sim_time < duration:
        torques = controller.update(dt)
        sim.apply_torques(torques)
        sim.step()
```

**Deliverable**: Walking mode integrated in main simulation

---

#### Task 5.2: Update Documentation
**Files**: README.md, QUICKSTART.md, STABILITY_IMPROVEMENT_PLAN.md
**Effort**: 2-3 hours
**Dependencies**: Task 5.1

**Updates**:
- Mark Phase 3 as complete
- Update walking mode status
- Add walking mode usage instructions
- Update success criteria with actual results

**Deliverable**: Updated documentation

---

#### Task 5.3: Create Phase 3 Session Summary
**File**: `SESSION_SUMMARY_PHASE3.md` (NEW)
**Effort**: 1-2 hours
**Dependencies**: All tasks complete

**Content**:
- Work completed
- Technical challenges and solutions
- Test results
- Performance metrics
- Lessons learned

**Deliverable**: Complete session summary

---

## Task Dependency Graph

```
Milestone 1 (Contact State Machine)
├── Task 1.1: Contact State Machine ──┐
└── Task 1.2: Contact Detection ──────┤
                                      │
Milestone 2 (WBC Walking Tasks)       │
├── Task 2.1: Swing Foot Task ────────┤
├── Task 2.2: Stance Foot Constraint ─┤
└── Task 2.3: Integration ────────────┴─→ Milestone 3

Milestone 3 (Contact Transitions)
├── Task 3.1: Smooth Transitions ─────┐
└── Task 3.2: Safety Checks ──────────┴─→ Milestone 4

Milestone 4 (Gait Tuning)
├── Task 4.1: Conservative Gait ──────┐
├── Task 4.2: Incremental Tuning ─────┤
└── Task 4.3: Test Suite ─────────────┴─→ Milestone 5

Milestone 5 (Integration)
├── Task 5.1: Main Integration ───────┐
├── Task 5.2: Documentation ──────────┤
└── Task 5.3: Session Summary ────────┘
```

---

## Phase 3 Success Criteria

### Minimum Viable (Must Achieve)
- ✅ Robot takes 10+ consecutive steps without falling
- ✅ Walking speed: >0.10 m/s
- ✅ ZMP stays within support polygon >90% of time
- ✅ Contact transitions without jerking

### Target (Should Achieve)
- ✅ Walking speed: >0.15 m/s
- ✅ ZMP stays within support polygon >95% of time
- ✅ Smooth, natural-looking gait
- ✅ Energy efficient (low torque²)

### Stretch (Nice to Have)
- ✅ Walking speed: >0.20 m/s
- ✅ Handles small external pushes (<10N)
- ✅ Adaptive step length/timing
- ✅ Stair-ready kinematics

---

## Risk Assessment

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| QP infeasibility during transitions | Medium | High | Add slack variables, tune transition duration |
| Foot slip on stance | Medium | High | Increase friction coefficient, add slip detection |
| CoM velocity too high | Low | Medium | Start with slow gait, tune MPC weights |
| Torque saturation | Medium | Medium | Monitor torques, reduce gait speed if needed |
| Swing foot collision | Low | Low | Conservative step height initially |

---

## Development Strategy

### Incremental Approach
1. ✅ Build on working WBC standing mode (Phase 2)
2. ✅ Start with single step, then extend
3. ✅ Conservative parameters first, optimize later
4. ✅ Extensive logging and visualization
5. ✅ Abort on first sign of instability

### Testing Strategy
1. **Unit Tests**: Each component in isolation
2. **Integration Tests**: Combined system
3. **Simulation Tests**: Full walking scenarios
4. **Regression Tests**: Ensure standing still works
5. **Performance Tests**: Speed, efficiency metrics

### Debug Tools Needed
- Real-time ZMP visualization
- Contact force monitoring
- Task priority visualization
- Gait phase indicator
- Torque saturation warnings

---

## Timeline Estimate

**Based on Phase 1 & 2 Performance (5x faster than estimated)**:

| Milestone | Original Estimate | Adjusted Estimate | Deliverables |
|-----------|-------------------|-------------------|--------------|
| M1: Contact State Machine | 2 days | 0.5 days | Contact FSM, detection |
| M2: WBC Walking Tasks | 2 days | 0.5 days | Swing/stance tasks, integration |
| M3: Contact Transitions | 1 day | 0.25 days | Smooth transitions, safety |
| M4: Gait Tuning | 2 days | 0.5 days | Tuned parameters, test suite |
| M5: Integration | 1 day | 0.25 days | Main integration, docs |
| **Total** | **8 days** | **2 days** | **Complete walking mode** |

**Confidence**: Medium-High
- Phase 1 & 2 established 5x velocity
- Good foundation from previous phases
- Well-defined architecture
- Clear success criteria

---

## Files to Create/Modify

### New Files (6)
1. `src/contact_state_machine.py` (~200 lines)
2. `src/test_contact_state_machine.py` (~100 lines)
3. `src/test_wbc_walking.py` (~300 lines)
4. `SESSION_SUMMARY_PHASE3.md` (~500 lines)
5. `PHASE3_WALKING_PLAN.md` (THIS FILE)

### Modified Files (5)
1. `src/wbc_tasks.py` - Add swing/stance tasks
2. `src/wbc_walking_controller.py` - Complete redesign
3. `src/main_simulation.py` - Integrate new walking controller
4. `README.md` - Update walking status
5. `QUICKSTART.md` - Add walking instructions
6. `STABILITY_IMPROVEMENT_PLAN.md` - Mark Phase 3 complete

---

## Next Steps

1. ✅ Review and approve this plan
2. ✅ Create development branch: `git checkout -b phase3-walking`
3. ✅ Start with Milestone 1 (Contact State Machine)
4. ✅ Iterate through milestones sequentially
5. ✅ Test continuously, commit frequently
6. ✅ Update documentation as you go

---

## Questions to Resolve Before Starting

1. **Control Frequency**: Use 30Hz (like MPC) or higher?
   - **Recommendation**: 30Hz to start, can increase if needed

2. **Torque vs Position Control**: Use inverse dynamics torques or convert to positions?
   - **Recommendation**: Try torques first (more direct), fallback to positions if unstable

3. **MPC Integration**: Use MPC for CoM planning or simple feedforward?
   - **Recommendation**: Simple feedforward initially, add MPC if needed

4. **Initial Test**: GUI or headless?
   - **Recommendation**: GUI for first tests to visualize, then headless for speed

---

**Document Status**: ✅ Ready for Review
**Next Action**: Get approval and start implementation
**Estimated Start**: After Phase 3 plan approval
**Estimated Completion**: 1-2 days after start

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Hunter is a **bipedal robot simulation** project using PyBullet. The robot is a 10-DOF (degrees of freedom) bipedal robot with 5 joints per leg. The project implements multiple control strategies with varying levels of sophistication and completion.

**Current Status (Phase 1 & 2 Complete, Phase 3 Investigation Complete):**
- ✅ **Standing mode**: Perfectly stable (Roll=0.2°, Pitch=0.1°) using PD control
- ✅ **Standing-MPC mode**: Stable using Model Predictive Control + ZMP
- ✅ **WBC mode**: Phase 2 complete - standing stability Roll=0.00°, Pitch=0.03°
- 🔍 **Walking mode**: Phase 3 investigation complete - WBC-hybrid control incompatibility identified, architectural redesign plan created (see WBC_ARCHITECTURAL_REDESIGN.md)

## Critical Knowledge

### Robot Configuration

**CRITICAL**: The robot requires precise initial configuration to maintain stability:

```python
# Correct base height (measured from URDF kinematics)
base_height = 0.679  # meters - DO NOT CHANGE without recalculating

# Stable standing configuration (straight legs, symmetric stance)
standing_config = {
    'leg_l1_joint': -0.1,  # Left hip roll (outward)
    'leg_l2_joint': 0.0,   # Left hip yaw
    'leg_l3_joint': 0.0,   # Left hip pitch (straight)
    'leg_l4_joint': 0.0,   # Left knee (straight)
    'leg_l5_joint': 0.0,   # Left ankle
    'leg_r1_joint': 0.1,   # Right hip roll (outward)
    'leg_r2_joint': 0.0,   # Right hip yaw
    'leg_r3_joint': 0.0,   # Right hip pitch (straight)
    'leg_r4_joint': 0.0,   # Right knee (straight)
    'leg_r5_joint': 0.0,   # Right ankle
}
```

**Why this matters:**
- This configuration provides **passive stability** - the robot can stand indefinitely with minimal control effort
- Deviating from `base_height=0.679m` causes ground contact issues and immediate instability
- Straight legs (all pitch/knee angles = 0) minimize required joint torques
- See `STABILITY_FIX.md` for detailed technical analysis

### Joint Naming Convention

Each leg has 5 joints (left: `leg_l*_joint`, right: `leg_r*_joint`):
- `leg_*1_joint`: Hip roll (lateral movement)
- `leg_*2_joint`: Hip yaw (rotation)
- `leg_*3_joint`: Hip pitch (forward/back)
- `leg_*4_joint`: Knee pitch
- `leg_*5_joint`: Ankle pitch

Total: **10 actuated joints**

## Common Commands

### Running Simulations

```bash
# Basic standing test (most stable, recommended for validation)
cd src
python main_simulation.py --mode standing --duration 10

# MPC-based standing (with active balance control)
python main_simulation.py --mode standing-mpc --duration 10

# WBC standing test (Phase 2 - advanced control)
python main_simulation.py --mode wbc --duration 10

# Walking mode (Phase 3 - in development)
python main_simulation.py --mode walking --duration 5

# Run without GUI (faster, headless)
python main_simulation.py --mode standing --duration 10 --no-gui
```

### Testing

```bash
# Test all modes at once
./scripts/test_all_modes.sh

# Individual Phase 1 & 2 tests
cd src
python test_stability_metrics.py      # Phase 1: CoM/ZMP calculation
python test_gravity_compensation.py   # Phase 1: Gravity compensation
python test_inverse_dynamics.py       # Phase 2: Dynamics validation
python test_phase1_integration.py     # Phase 1: Integration test
python test_wbc_standing.py           # Phase 2: WBC validation
```

### Docker Environment

```bash
# Start Docker container
cd docker
docker-compose up -d

# Run simulation in Docker
docker exec hunter-simulation bash /workspace/hunter/scripts/test_all_modes.sh

# Shell into container
docker exec -it hunter-simulation bash
```

### Configuration

Configuration is in `config/default_config.yaml`. Key parameters:
- `simulation.dt`: Timestep (default: 0.001s = 1kHz)
- `gait.*`: Gait parameters (step length, height, period)
- `pd_controller.joint_gains`: Per-joint PD gains

## Architecture Overview

### Control System Hierarchy

The project implements two fundamentally different control approaches:

#### 1. Simple Kinematic Controller (Functional ✅)

**Architecture:**
```
GaitGenerator → InverseKinematics → PDController → Robot
```

**Files:**
- `gait_generator.py`: Generates time-based foot trajectories
- `inverse_kinematics.py`: PyBullet IK solver (foot position → joint angles)
- `pd_controller.py`: Joint-level PD control (angles → torques)
- `main_simulation.py`: Orchestrates the control loop

**Status:** Works for standing mode. **NOT suitable for walking** due to architectural limitation - PyBullet IK assumes fixed base, but bipedal walking has a free-floating base.

#### 2. Advanced Dynamic Controller (Phase 1 & 2 Complete ✅)

**Architecture:**
```
MPC (high-level planner) → WBC (force optimization) → InverseDynamics → Robot
```

**Files:**
- `mpc_controller.py`: Model Predictive Control for CoM trajectory planning
- `wbc_controller.py`: Whole-Body Control via Quadratic Programming (QP)
- `wbc_tasks.py`: Task hierarchy management for WBC
- `inverse_dynamics.py`: Maps desired accelerations to joint torques (τ = M(q)q̈ + g(q))
- `mpc_wbc_controller.py`: Integrates MPC + WBC

**Key Modules (Phase 1 & 2):**
- `stability_metrics.py`: Accurate CoM/ZMP computation (16.7cm improvement over base-only)
- `gravity_compensation.py`: Feedforward gravity torques (30% efficiency gain)
- `inverse_dynamics.py`: Mass matrix M(q), gravity torques g(q)

**Status:**
- ✅ Phase 1 Complete: Core stability fundamentals (CoM, ZMP, gravity compensation)
- ✅ Phase 2 Complete: WBC tuned for standing (Roll=0.00°, Pitch=0.03°, force error 0.1%)
- 🚧 Phase 3 Pending: Walking mode requires contact-aware WBC architecture

### Key Architectural Insights

1. **Why IK-based walking doesn't work:**
   - PyBullet's `calculateInverseKinematics()` assumes the robot base is fixed
   - In bipedal walking, the base is free-floating (not fixed to ground)
   - This causes large errors (50-250cm) during swing phase
   - Solution: Use WBC which respects free-floating dynamics

2. **MPC + WBC Synergy:**
   - **MPC**: High-level planner, computes optimal CoM trajectory over prediction horizon
   - **WBC**: Low-level executor, translates CoM commands into joint torques while satisfying physical constraints (friction, torque limits)
   - This separation allows predictive planning (MPC) + reactive execution (WBC)

3. **Phase 1 & 2 Achievements:**
   - CoM calculation now uses all links (not just base) → 16.7cm accuracy improvement
   - ZMP includes acceleration terms: `zmp_x = x - (h/g) * ddot_x` (dynamic, not just static projection)
   - Gravity compensation reduces required control torques by ~30%
   - Inverse dynamics enables proper torque control: τ = M(q)q̈ + g(q)
   - WBC QP optimization achieves 0.1% force tracking error

## Development Workflow

### Adding New Control Features

1. **For standing/balance improvements:**
   - Modify `balance_controller.py` or `mpc_wbc_controller.py`
   - Tune parameters in `config/default_config.yaml` or directly in controller params
   - Test with: `python main_simulation.py --mode standing-mpc --duration 10`

2. **For WBC improvements:**
   - Modify QP formulation in `wbc_controller.py`
   - Adjust task weights in `WBCParams` dataclass
   - Validate with: `python test_wbc_standing.py`
   - Current tuned parameters (Phase 2):
     ```python
     friction_coef = 0.6              # Ground friction
     w_force_tracking = 10.0          # Force tracking weight (10x default)
     w_force_regularization = 0.01    # Force smoothing (reduced from 0.1)
     ```

3. **For Phase 3 walking development:**
   - Start with `wbc_walking_controller.py` (currently incomplete)
   - Implement contact state machine (double support, left swing, right swing)
   - Add swing foot trajectory tracking task to WBC
   - Handle contact transitions smoothly (heel strike, toe off)
   - See `STABILITY_IMPROVEMENT_PLAN.md` Phase 3 section for detailed plan

### Debugging Stability Issues

**If robot falls in standing mode:**
1. Verify `base_height = 0.679` in `main_simulation.py`
2. Check straight-leg configuration is being used
3. Inspect PD gains - recommended: `Kp=200-300`, `Kd=20-30`
4. Use diagnostic tool: `python diagnostics/find_stable_pose.py`

**If WBC mode is unstable:**
1. Check QP solver status (should be 100% feasible)
2. Verify force balance: `total_GRF ≈ robot_weight` (should be within 5N)
3. Check ZMP is inside support polygon
4. Tune `w_force_tracking` and `friction_coef` parameters

**If walking mode fails:**
- Expected behavior - architectural limitation of current IK-based approach
- See `WALKING_MODE_INVESTIGATION.md` for technical analysis
- Phase 3 will replace with WBC-based walking

### Validation and Testing

**After making control changes, always run:**
```bash
# Quick validation (5 seconds each mode)
./scripts/test_all_modes.sh

# Extended validation (specific mode)
python main_simulation.py --mode standing --duration 30
```

**Success criteria:**
- Standing: Roll < 5°, Pitch < 5°, Height ≈ 0.69m
- Standing-MPC: Same as standing
- WBC: Roll < 1°, Pitch < 1° (Phase 2 target exceeded: actual Roll=0.00°, Pitch=0.03°)
- Walking: Currently maintains standing position (Phase 3 will enable actual walking)

## Important Files and Documentation

### Technical Documentation
- `README.md`: Comprehensive user guide, setup instructions, current status
- `STABILITY_IMPROVEMENT_PLAN.md`: **4-phase development plan** (Phase 1 & 2 complete)
- `CONTROL_SYSTEM_OVERVIEW.md`: High-level explanation of control architectures
- `STABILITY_FIX.md`: Technical details on how standing stability was achieved
- `WALKING_MODE_INVESTIGATION.md`: Analysis of walking mode limitations
- `MPC_WALKING_FIX.md`: MPC controller tuning and fixes
- `ARCHITECTURE_CHANGES_SUMMARY.md`: History of architectural evolution

### Core Source Files
- `src/main_simulation.py`: Entry point, defines all test modes
- `src/simulation_env.py`: PyBullet interface wrapper
- `src/pd_controller.py`: Basic PD control with per-joint gains
- `src/balance_controller.py`: MPC + ZMP balance control
- `src/mpc_controller.py`: Model Predictive Control implementation
- `src/wbc_controller.py`: Whole-Body Control via QP optimization
- `src/inverse_dynamics.py`: Robot dynamics (M(q), g(q)) - Phase 2
- `src/stability_metrics.py`: CoM/ZMP calculation - Phase 1
- `src/gravity_compensation.py`: Feedforward gravity torques - Phase 1

### Diagnostic Tools
- `src/diagnostics/find_stable_pose.py`: Analyzes foot placement and base height
- `src/test_*.py`: Unit tests for individual modules

## Phase 1 & 2 Accomplishments

**Completed November 23-24, 2025** (4 days vs 4 weeks estimated - 5x faster!)

### Phase 1: Core Stability Fundamentals ✅
- Accurate CoM calculation using all links → **16.7cm improvement**
- Dynamic ZMP computation with acceleration terms
- Gravity compensation → **30% torque efficiency gain**
- All modules tested and integrated

### Phase 2: WBC Tuning & Validation ✅
- WBC standing: **Roll=0.00°, Pitch=0.03°** (far exceeds <1° target)
- Force optimization: **0.1% error** (excellent accuracy)
- Inverse dynamics: τ = M(q)q̈ + g(q) fully implemented
- QP solver: 100% feasible, no convergence issues

### Next: Phase 3 - Walking Mode 🚧
- Redesign `wbc_walking_controller.py` with contact-aware WBC
- Implement swing foot trajectory tracking
- Handle contact state transitions (double support ↔ single support)
- Target: 10+ consecutive steps without falling

## Notes for Future Development

1. **Never modify the base height** (`0.679m`) without recalculating from URDF kinematics
2. **Always use straight-leg configuration** as the baseline for standing tests
3. **PyBullet's IK is unsuitable for walking** - use WBC with contact constraints instead
4. **Phase 3 walking requires:**
   - Contact state machine
   - Swing foot trajectory task in WBC
   - Smooth contact transition handling
   - See `STABILITY_IMPROVEMENT_PLAN.md` Phase 3 section for detailed plan
5. **WBC QP tuning** is sensitive to weight ratios - when changing weights, adjust gradually
6. **Free-floating base dynamics** require special handling in PyBullet functions (they often return 16x16 matrices: 6 base DOF + 10 joints)

## Repository Organization

```
hunter/
├── src/                    # Source code
│   ├── main_simulation.py  # Entry point
│   ├── *_controller.py     # Various control strategies
│   ├── simulation_env.py   # PyBullet wrapper
│   ├── stability_metrics.py # Phase 1 - CoM/ZMP
│   ├── gravity_compensation.py # Phase 1
│   ├── inverse_dynamics.py # Phase 2
│   ├── test_*.py          # Unit tests
│   └── diagnostics/        # Analysis tools
├── config/                 # YAML configuration
├── models/urdf/           # Robot model
├── scripts/               # Bash scripts
├── logs/                  # Runtime logs
└── *.md                   # Documentation

Key docs:
- README.md (start here)
- STABILITY_IMPROVEMENT_PLAN.md (development roadmap)
- CONTROL_SYSTEM_OVERVIEW.md (architecture)
```

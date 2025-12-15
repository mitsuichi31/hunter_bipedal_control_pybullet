# hunter_bipedal_control_pybullet — Reconstruction Plan

Goal: Refactor the PyBullet stack to mirror the ROS1 container’s MPC → state estimation → WBC pipeline so Hunter can achieve stable standing and walking in simulation.

## Target Architecture (parity with container)
- Simulation core (`simulation_env.py`): thin HAL that provides sensor streams (base pose/vel, contacts, joint states) and accepts hybrid joint commands (pos/vel/Kp/Kd + torque). Physics params (friction, ERP/CFM) are config-driven.
- Estimation: simple observer with low-pass/EMA on base + joints and contact detection with hysteresis; mimics ROS estimator outputs.
- Reference & gait: config-driven gait schedule (stance/trot variants) that produces contact sequences and nominal foot placements; parameters aligned with container `gait.info`/`reference.info`.
- MPC thread (~100 Hz): centroidal dynamics MPC producing CoM trajectories, contact schedule, and desired contact forces.
- WBC loop (per sim step, ~1 kHz): QP tracking MPC outputs with task hierarchy and friction constraints; outputs hybrid joint commands. PD fallback on infeasibility.
- Safety: torque/velocity clamps, torso-angle bounds; structured logging of QP status and contact states.

## Module/Layout Changes
- `src/estimation/` (new):
  - `state_filter.py`: EMA/low-pass filters for base and joint states.
  - `contact_estimator.py`: contact detection via force threshold + hysteresis.
  - `observer.py`: wraps filters and contact estimation into a single state output.
- `src/planning/` (new):
  - `gait_schedule.py`: phase generator matching `gait.info` patterns (stance/trot/standing_trot).
  - `reference_manager.py`: target velocities/COM height management.
  - `centroidal_mpc.py`: centroidal MPC (extends/replaces `mpc_controller.py`) returning CoM + contact forces/schedule.
- `src/control/` (refactor):
  - `wbc_controller.py`: QP with explicit task hierarchy (base pose/vel > contact force tracking > swing foot > joint regularization), friction cones, normal-force bounds, torque/vel limits.
  - `wbc_tasks.py`: task builders aligned to the hierarchy above.
  - `mpc_wbc_controller.py`: orchestrates MPC outputs → WBC inputs with explicit timing; removes ad-hoc coupling.
  - `pd_controller.py`: kept as fallback; expose hybrid command interface.
- `simulation_env.py`: exposes `get_observations()` and `apply_hybrid_command()` with clamps; loads physics params from config.
- `config/`:
  - `gait.yaml`: phase durations, swing heights, duty factors, step length/speed.
  - `reference.yaml`: COM height, velocity targets.
  - `task.yaml`: MPC/WBC weights, friction coefficient, force/torque limits, safety bounds.
- `diagnostics/`:
  - `baseline_recorder.py`: logs Roll/Pitch, foot forces, torques for standing/trot; pass/fail vs thresholds.
  - `qp_monitor.py`: records QP feasibility, force/torque ranges.

## Control Loop (concept)
- MPC (100 Hz): read filtered state → update gait phase → solve centroidal MPC → produce CoM traj + desired contact forces + schedule.
- WBC (per sim step): read filtered state + contact flags → build tasks from MPC outputs → solve QP → apply hybrid command; fallback to PD if infeasible.
- Contact handling: detector provides contact flags and estimated forces; friction cones in controller match PyBullet friction.

## Implementation Steps
1) Config + plumbing
   - Add `gait.yaml`, `reference.yaml`, `task.yaml`; update `config_loader.py`.
   - Refactor `simulation_env.py` to apply physics params and provide observation/command APIs.
2) Estimation
   - Implement `estimation/state_filter.py` and `contact_estimator.py`; integrate via `observer.py` in `main_simulation.py`.
3) MPC
   - Rework `mpc_controller.py` into `planning/centroidal_mpc.py` with centroidal dynamics, short horizon; output CoM + contact forces/schedule.
4) WBC
   - Rewrite `wbc_controller.py`/`wbc_tasks.py` with explicit hierarchy, friction cones, force bounds, torque/vel clamps; add PD fallback and QP logging.
5) Orchestration
   - Update `mpc_wbc_controller.py` to synchronize MPC (10 ms) and WBC (per step), consuming observer outputs; update `main_simulation.py` to select modes (`standing`, `standing-mpc`, `trot`/`walking`).
6) Tests/diagnostics
   - Extend `scripts/test_all_modes.sh` for standing/trot bouts with acceptance criteria (no fall, torso angle bounds); record logs.
7) Parity with ROS container (cross-check)
   - Pull COM height, mass/inertia, friction coefficient, and gait timings from `hunter_bipedal_control_container/legged_controllers/config/hunter/{task.info,gait.info,reference.info}` and map into the new YAMLs.

## Success Criteria
- Standing and trot modes pass baseline recorder checks (torso angles bounded, no falls, feasible QP).
- MPC+WBC walking completes short bouts without falls in simulation under nominal settings.
- Configs capture all tuned parameters; controller logs make failures diagnosable (QP status, forces, torques).

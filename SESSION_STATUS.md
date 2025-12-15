# Session Status (feature/pybullet-rebuild-plan)

Date: 2025-xx-xx (update when resuming)

## What we did this session
- Created branch `feature/pybullet-rebuild-plan` and merged `origin/phase4-position-control-walking` to pull in Phase 4 walking work, COM planner, contact state machine, and expanded tests/docs.
- Added reconstruction guidance: `REBUILD_PLAN.md`, `AGENTS.md`, `AGENTS_CONTAINER.md`.
- Scaffolding toward ROS-like MPC→estimation→WBC pipeline:
  - New configs with ROS parity: `config/gait.yaml`, `config/reference.yaml`, `config/task.yaml`.
  - Config loader extended to parse gait/reference/task YAMLs while keeping legacy defaults.
  - Estimation skeleton: low-pass filters, contact estimator with hysteresis, observer wrapper (`src/estimation/...`).
  - Planning skeleton: gait schedule helper, reference manager, centroidal MPC placeholder (`src/planning/...`).

## Current branch state
- Branch: `feature/pybullet-rebuild-plan`
- Working tree: clean (last commit: scaffold configs + estimation/planning skeletons)

## Next steps (resume here)
1) Integrate new configs/observer into runtime:
   - Wire `config_loader` outputs into `main_simulation.py` and `simulation_env.py`.
   - Add `get_observations()` and `apply_hybrid_command()` APIs to `simulation_env.py`, applying physics params from config (friction/ERP/CFM).
   - Use `Observer` to filter base/joint states and contact forces before controllers consume them.
2) Planning/MPC:
   - Replace centroidal MPC placeholder with usable planner or couple to existing COM planner from Phase 4; ensure outputs (CoM traj + contact forces + schedule) align with `task.yaml` weights.
3) WBC alignment:
   - Refactor `wbc_controller.py`/`wbc_tasks.py` to use new task weights, friction coefficient, normal force bounds, and torque/velocity limits; add PD fallback on infeasible QP.
4) Orchestration:
   - Update `mpc_wbc_controller.py` to synchronize MPC (10 ms) and WBC (per-step) using observer outputs; expose modes in `main_simulation.py` (`standing`, `standing-mpc`, `walking/trot`).
5) Tests/diagnostics:
   - Extend `scripts/test_all_modes.sh` to include standing/trot with torso-angle bounds; hook in baseline recorder once added.

## Open questions / notes
- Decide how much of the Phase 4 position-control walking to keep vs. supersede with MPC→WBC; may keep as a fallback mode.
- Confirm friction/COM height/mass consistency between PyBullet URDF and container configs when tuning.

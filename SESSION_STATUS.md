# Session Status (feature/pybullet-rebuild-plan)

Date: 2025-12-15

## What we did this session
- Wired config/observer/runtime plumbing:
  - `simulation_env.py` now exposes `get_observations()`/`apply_hybrid_command()`, applies physics params, and discovers foot links.
  - `main_simulation.py` consumes `gait/reference/task` YAMLs, filters state via `Observer`, and auto-enables enhanced contacts when commanding forward velocity.
  - Added GUI MIT-SHM/OpenGL notes to README/run scripts.
- Planning/WBC:
  - Upgraded centroidal planner: gait-driven contact schedule, nominal forces, mass aligned to URDF (12.6 kg).
  - MPCWBC now accepts filtered observations, planner contacts/forces, and enforces total normal force ≥ mg in WBC QP.
  - Auto enhanced-contact solver when forward velocity is requested.
- Stability:
  - WBC stable 10s with forward velocity 0.4 m/s gait trot; force tracking matches gravity.
  - Standing/standing-mpc/walking modes stable in GUI/headless runs.
- Tests:
  - Added regression `src/test_wbc_forward_velocity.py`; hooked pytest into `scripts/test_all_modes.sh`; pytest added to requirements; `test_all_modes` passes in container.
- Investigation:
  - Walking mode manual test shows sideways drift and little forward motion despite commands; needs tuning.

## Current branch state
- Branch: `feature/pybullet-rebuild-plan`
- Working tree: clean (last commit: test harness + WBC stability fixes)

## Next steps (resume here)
1) Walking forward drift: log base X/Y/gait targets in walking mode, inspect reference_x updates and footstep symmetry, retune gains/foot placements to recover forward progress.
2) Expand regressions: add higher-velocity WBC regression (e.g., 0.4 m/s trot) and optional walking smoke pytest.
3) Diagnostics: add CoM/support/ZMP logging and planner vs. solved force deltas; consider longer endurance runs (30–60s) in GUI/headless.
4) Parity check: ensure mass/friction/contact solver settings are consistent across standing-mpc/walking and document defaults.

## Open questions / notes
- Decide how much of the Phase 4 position-control walking to keep vs. supersede with MPC→WBC; may keep as a fallback mode.
- Confirm friction/COM height/mass consistency between PyBullet URDF and container configs when tuning.

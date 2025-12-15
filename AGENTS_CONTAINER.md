# PyBullet Code Map

Reference for the experimental PyBullet stack. Use this to navigate the codebase and plan future improvements.

## Quick Facts
- Purpose: fast iteration sandbox for Hunter posture/balance control; not production-grade and cannot reliably walk yet.
- Stack: pure Python (PyBullet) with lightweight controllers (PD, MPC+ZMP, experimental WBC via cvxpy).
- Entrypoint: `src/main_simulation.py` with modes `standing`, `standing-mpc`, `wbc`, `walking`.
- Config: `config/default_config.yaml` for gains, mass/geometry, and simulation constants.

## Layout
- `config/`: YAML defaults; tune controller gains, contact params, and simulation options here.
- `models/urdf/`: Hunter URDF + decimated meshes used in PyBullet.
- `src/`:
  - `main_simulation.py`: CLI front-end selecting controller mode; sets up simulation_env.
  - `simulation_env.py`: world creation, sensor/actuator plumbing, stepping loop.
  - Controllers:
    - `pd_controller.py`: basic joint-space PD for standing.
    - `balance_controller.py`: MPC+ZMP for upright balance.
    - `mpc_controller.py`: underlying MPC routines used by balance controller.
    - `wbc_controller.py` + `wbc_tasks.py`: QP WBC prototype (cvxpy).
    - `mpc_wbc_controller.py`: hybrid MPC→WBC attempt.
    - `wbc_walking_controller.py`: early walking research; unstable.
  - Motion helpers: `inverse_kinematics.py`, `gait_generator.py`.
  - Diagnostics: `diagnostics/` (e.g., `find_stable_pose.py`).
- `scripts/`: `run_simulation.sh`, `test_all_modes.sh`, Docker helper `run_docker.sh`.
- `docker-compose.yml` / `Dockerfile`: reproducible container (`hunter-sim` service) mounting repo to `/workspace/hunter`.
- Docs: README + topical notes (`STABILITY_FIX.md`, `MPC_WALKING_FIX.md`, `WALKING_MODE_INVESTIGATION.md`, `ARCHITECTURE_CHANGES_SUMMARY.md`, etc.).

## Control Pipeline (current)
1. `simulation_env` loads URDF into PyBullet and applies config values.
2. Chosen controller computes joint torques/targets:
   - PD: direct joint-space PD to hold a pose.
   - MPC+ZMP: plans CoM/foot ZMP trajectory, feeds PD targets.
   - WBC (prototype): builds a cvxpy QP with task weights (base pose, contact forces); not yet stable.
3. Commands are applied to PyBullet joints each step; no external estimator stack.

## Run/Test
- Docker (recommended): `docker-compose up -d` then `docker-compose exec hunter-sim bash`; run `bash scripts/test_all_modes.sh` or `python src/main_simulation.py --mode standing-mpc --duration 10 --no-gui`.
- Host: `pip install -r requirements.txt`; run the same `python src/main_simulation.py ...` commands.
- Logging: check `logs/` if scripts emit outputs there; most runs are console-driven.

## Known Limitations
- Walking is not reliable; WBC and MPC→WBC integration need tuning and better contact modeling.
- Estimation is simplistic (PyBullet ground truth); hardware-like noise/latency not modeled.
- Contact dynamics rely on PyBullet parameters; friction/cone consistency with controller is approximate.

## Future Work Ideas
- Stabilize WBC: adjust task weights/friction cones, add torque limits, and validate against standing baselines.
- Improve walking: better gait schedule + foot placement, integrate momentum constraints, and add preview horizon checks.
- Add regression harness: capture Roll/Pitch/foot forces for `standing`/`standing-mpc` to detect controller regressions.
- Align configs with ROS stack: reuse COM height, mass, and friction values to compare behaviors across simulators.

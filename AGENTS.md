# Repository Guidelines

## Project Structure & Module Organization
- Core code lives in `src/`; entry point is `src/main_simulation.py` with modes `standing`, `standing-mpc`, `wbc`, `walking`.
- Controllers: `pd_controller.py`, `balance_controller.py`, `mpc_controller.py`, `wbc_controller.py`, `wbc_tasks.py`, `mpc_wbc_controller.py`, in-progress `wbc_walking_controller.py`.
- Supporting modules: kinematics/planning (`inverse_kinematics.py`, `gait_generator.py`), dynamics/stability (`stability_metrics.py`, `gravity_compensation.py`, `inverse_dynamics.py`), contact logic (`contact_state_machine.py`).
- Tests sit next to code as `test_*.py`; diagnostics live in `src/diagnostics/` with its own README.
- Config: `config/default_config.yaml`; assets: `models/urdf/hunter.urdf`; scripts: `scripts/`; logs: `logs/`.

## Build, Test, and Development Commands
- Install: `pip install -r requirements.txt`.
- Docker (recommended): `cd docker && docker-compose up -d`; run suite headless via `docker exec hunter-simulation bash /workspace/hunter/scripts/test_all_modes.sh`.
- Local smoke: `python src/main_simulation.py --mode standing --duration 10 --no-gui`; swap `standing-mpc` or `wbc` for other validated modes.
- Targeted: from `src/`, `python test_wbc_standing.py`, `python test_inverse_dynamics.py`, etc. Add new tests as `test_<feature>.py`.

## Coding Style & Naming Conventions
- Python 3 with 4-space indentation; add type hints and brief docstrings for control assumptions or constraints.
- Use `snake_case` for modules/functions/variables, `CamelCase` for classes, `UPPER_SNAKE_CASE` for constants. Follow joint names (`leg_l1_joint` … `leg_r5_joint`) and config keys from `default_config.yaml`.
- Keep modes explicit in flags (`--mode standing-mpc`, `--no-gui`) and preserve existing CLI patterns when adding options.

## Testing Guidelines
- Regression bar: `scripts/test_all_modes.sh` must stay green for standing/standing-mpc/wbc; walking can be experimental but must not regress stable modes.
- Prefer headless runs (`--no-gui`) in CI-like checks; include expected Roll/Pitch snippets in PRs when control logic changes.
- Add a nearby `test_<feature>.py`; for dynamics, extend Phase 1/2 tests instead of creating ad hoc scripts.

## Commit & Pull Request Guidelines
- Mirror existing log style: short, imperative, with context (e.g., “Add active balance control (hip + ankle feedback)”, “Phase 3 M4.2: Gait Parameter Tuning & Diagnostics”).
- PRs should state scope (mode/phase touched), config changes (notably `config/default_config.yaml`), tests run with output snippets, and GUI evidence if behavior is visual.
- Call out any change to critical constants (e.g., base height) and justify it.

## Simulation Safety & Configuration Tips
- Maintain the canonical base height `0.679` m and straight-leg standing pose; changing either requires recalculating stability metrics.
- Adjust parameters in `config/default_config.yaml` instead of hardcoding; document new keys briefly in comments or PR notes.
- Use `scripts/run_simulation.sh` or Docker wrappers to keep working directories and asset paths consistent.

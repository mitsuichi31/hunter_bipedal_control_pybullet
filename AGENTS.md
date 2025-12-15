# Repository Guidelines

Contributor quickstart for the Hunter bipedal PyBullet simulation. Default to Docker for reproducibility and document any change that affects the stable standing modes.

## Project Structure & Module Organization
- `src/`: simulation code; `main_simulation.py` entrypoint, controllers in `*_controller.py`, diagnostics in `src/diagnostics/` (see its README).
- `config/`: YAML configs such as `default_config.yaml`; do not hide parameters in code.
- `models/`: URDF and decimated meshes for the Hunter robot; keep assets small.
- `scripts/`: workflow helpers (`run_docker.sh`, `run_simulation.sh`, `test_all_modes.sh`).
- `docker-compose.yml`/`Dockerfile`: reproducible container; repo mounts to `/workspace/hunter` inside `hunter-simulation`.

## Build, Test, and Development Commands
- Install deps locally: `pip install -r requirements.txt` from repo root.
- Docker flow (recommended): `docker-compose up -d` then `docker-compose exec hunter-sim bash`.
- Quick run (host): `python src/main_simulation.py --mode standing --duration 5 --no-gui`; in-container, same path under `/workspace/hunter`.
- Wrapper: `bash scripts/run_simulation.sh standing 10 no-gui` (in container) to exercise a mode.
- Smoke test suite: `bash scripts/test_all_modes.sh` (standing/standing-mpc should pass; wbc/walking instability is expected—note that in PRs).

## Coding Style & Naming Conventions
- Python 3; follow PEP 8 with 4-space indents, snake_case functions/variables, CapWords classes.
- Use type hints and docstrings for new public functions; keep controller modules named `*_controller.py` and diagnostics under `src/diagnostics/`.
- Keep configs in YAML, not constants; avoid magic numbers for base height or gains.

## Testing Guidelines
- Default gate is `scripts/test_all_modes.sh --no-gui`; capture Roll/Pitch output for standing modes as a regression check.
- Keep new diagnostics short (<10s) and place them in `src/diagnostics/` with a brief README note.
- For new modes, provide a minimal command to run them and document expected stable ranges.

## Commit & Pull Request Guidelines
- Commits: short, imperative summaries with scope and rationale (e.g., `Fix URDF visualization issue: decimate meshes`). Group related changes.
- PRs: include description, commands run, observed outputs (Roll/Pitch or distance), and known limitations. Link issues/tasks and call out config/URDF edits explicitly.
- Run `test_all_modes.sh` before merging; highlight any intentional failures.

## Configuration & Safety Notes
- Do not alter critical stability constants (notably base height) without benchmarks; record before/after values when you do.
- Keep meshes/assets lightweight to avoid slow docker rebuilds.
- Avoid committing local secrets or display settings; prefer environment variables passed through docker-compose.

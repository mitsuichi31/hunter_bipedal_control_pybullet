# Next Tasks Checklist

- [ ] Sanity-check stable modes: `bash scripts/test_all_modes.sh` (or Docker wrapper) to confirm standing/standing-mpc/wbc stay green before changes.
- [ ] Capture current walking baseline: `python src/test_walking_baseline.py --no-gui` to regenerate `test_results_*.json/md` as pre-change reference.
- [ ] Integrate full torque path in `wbc_walking_controller.py`: drive torques from WBC QP + inverse dynamics instead of joint-space PD; keep contact transitions in `contact_state_machine.py` consistent.
- [ ] Retune gait parameters (step length/height/period, double-support) for stability targets (>5s balance) and log deltas in `PHASE3_WALKING_PLAN.md`.
- [ ] Add minimal regression hook: a short headless walking smoke (e.g., 2s) wired into `scripts/test_all_modes.sh` once torque path is stable.
- [ ] Update docs after tuning: summary in `WALKING_MODE_INVESTIGATION.md` and brief note in `AGENTS.md` if workflow/commands change.

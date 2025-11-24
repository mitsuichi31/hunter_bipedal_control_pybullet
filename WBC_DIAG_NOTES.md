# WBC Walking Diagnostics (Current Status)

Date: 2025-11-24  
Scope: Walking mode stability investigation with `WALKING_WBC=1`

## What Was Tried
- Kept walking mode stable by default via standing `POSITION_CONTROL`. Added `WALKING_WBC` env toggle to enable WBC stepping on top of the posture baseline.
- Instrumented `wbc_walking_controller.py` joint order and torque mapping; confirmed actuated order matches joint names: `leg_l1_joint … leg_r5_joint`.
- Switched WBC update rate to 1 kHz (control_dt=0.001) and tested posture-only torque paths (gravity comp + posture PD) with frozen contacts and ZMP checks disabled.
- Added torque clamps (first ±200 Nm, then ±50 Nm, then ±20 Nm) and reduced posture PD gains progressively (200/20 → 100/10 → 50/5, with hip/knee pitch up to 80/8).
- Tried running WBC with and without position control motors active; position control keeps stability when WBC is off, but turning it off for WBC still leads to blow-up.
- Frozen contact state (double support) and zero-step gait targets during diagnostics to remove FSM/gait influence.

## Findings
- Default walking (POSITION_CONTROL only) is stable (roll≈0.2°, pitch≈−0.1° over 10s).
- Enabling WBC stepping causes immediate instability even with zero stepping and frozen contacts. Torques quickly hit clamps and roll/pitch diverge (up to ~180°/90°).
- Joint order mapping is correct; the issue is not a name mismatch.
- Torque magnitudes saturate despite aggressive clamping and low gains, suggesting posture PD alone is not sufficient in the free-floating WBC loop (likely missing damping/force distribution and conflicting with the base dynamics).
- Inverse dynamics warnings disappeared after using gravity_comp only; instability remains due to control behavior, not joint ordering.

## Current State
- `WALKING_WBC=0` (default): walking uses standing `POSITION_CONTROL` and is stable.
- `WALKING_WBC=1`: WBC posture-only torques still unstable; torques clamp at ±20 Nm but robot tips within ~2 s.
- Uncommitted changes during diagnostics were reverted except for the WBC toggle; the latest experiments were left uncommitted in `src/wbc_walking_controller.py` when testing.

## Suggested Next Steps
1) Revert `wbc_walking_controller.py` to the last committed stable baseline when enabling WBC; keep position control active while gradually layering a minimal torque overlay (e.g., very low gains, high damping).
2) Add heavy damping and start with zero-step gait and frozen contacts; reintroduce gait and safety checks incrementally only after demonstrating posture-hold with torques.
3) Log base height/CoM and applied torques for the first few steps to detect bad initial conditions; confirm base height stays near 0.679 m.
4) Keep WBC off by default in walking until a stable torque path is validated.

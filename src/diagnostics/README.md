# Diagnostic Tools

This directory contains diagnostic and analysis tools used during development and investigation. These tools are preserved for reference and future debugging.

## Stability Analysis

### find_stable_pose.py
**Purpose**: Analyze robot configurations to find stable standing poses

**Usage**:
```bash
python3 diagnostics/find_stable_pose.py
```

**Output**:
- Foot height asymmetry analysis
- Ground contact verification
- Base height calculations
- Configuration comparisons

**Status**: ✅ Useful for pose tuning and stability analysis

---

## Walking Mode Investigation Tools

The following tools were created during the walking mode investigation (Nov 23, 2025). The investigation identified fundamental architectural limitations requiring inverse dynamics library integration. These tools are preserved for reference.

### diagnose_walking_bug.py
**Purpose**: Analyze gait generator output and coordinate transformations

**Created**: Nov 23, 2025
**Investigation**: Walking mode coordinate frame bug
**Result**: ✅ Fixed coordinate frame feedback loop in main_simulation.py

**Usage**:
```bash
python3 diagnostics/diagnose_walking_bug.py
```

**What it does**:
- Analyzes gait generator trajectories
- Verifies coordinate transformations
- Detects feedback loops in reference frames

### test_walking_detailed.py
**Purpose**: Real-time walking simulation diagnostics

**Created**: Nov 23, 2025
**Investigation**: Walking mode IK accuracy and foot placement
**Result**: ✅ Identified IK/free-floating base incompatibility

**Usage**:
```bash
python3 diagnostics/test_walking_detailed.py
```

**What it does**:
- Measures IK foot placement accuracy
- Tracks base position over time
- Monitors walking stability

### test_ik_walking.py
**Purpose**: Isolated IK solver testing for walking motions

**Created**: Nov 23, 2025
**Investigation**: IK solver behavior with free-floating base
**Result**: ✅ Confirmed PyBullet IK assumes fixed base

**Usage**:
```bash
python3 diagnostics/test_ik_walking.py
```

**What it does**:
- Tests IK solver in isolation
- Compares target vs achieved foot positions
- Analyzes IK convergence

---

## Investigation Summary

For complete details of the walking mode investigation and findings, see:
- [WALKING_MODE_INVESTIGATION.md](../../WALKING_MODE_INVESTIGATION.md)
- [ARCHITECTURE_CHANGES_SUMMARY.md](../../ARCHITECTURE_CHANGES_SUMMARY.md)

**Key Findings**:
- Walking requires inverse dynamics (Pinocchio library)
- PyBullet IK assumes fixed base, incompatible with free-floating bipeds
- Current architecture can achieve perfect standing (Roll=0.2°) but not walking

---

**Note**: These tools are preserved for reference and future development. They successfully identified and documented the architectural requirements for walking mode implementation.

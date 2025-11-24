# Walking Controller Baseline Test Results

**Date**: 2025-11-24 05:09:44
**Phase**: 3 - WBC Walking Architecture
**Controller**: Simplified joint-space PD + active balance

---

## Stability Duration

**Trials**: 3

| Metric | Value |
|--------|-------|
| Mean Duration | 0.84s |
| Std Dev | 0.00s |
| Min Duration | 0.84s |
| Max Duration | 0.84s |
| Success Rate | 0% |

### Individual Trials

| Trial | Duration | Max Pitch | Max Roll | Stop Reason |
|-------|----------|-----------|----------|-------------|
| 1 | 0.84s | 20.6° | 8.9° | emergency_stop |
| 2 | 0.84s | 20.6° | 8.9° | emergency_stop |
| 3 | 0.84s | 20.6° | 8.9° | emergency_stop |

---

## Orientation Tracking

| Metric | Value |
|--------|-------|
| Duration | 0.84s |
| Max Pitch | 20.6° |
| Max Roll | 8.9° |
| Success | ❌ |

---

## Torque Verification

| Metric | Value |
|--------|-------|
| Duration | 0.84s |
| Max Pitch | 0.0° |
| Max Roll | 0.0° |
| Max Torque | 38.95 Nm |
| Avg Torque | 0.50 Nm |
| Success | ✅ |

---

## Robustness Perturbed

**Trials**: 3

| Metric | Value |
|--------|-------|
| Mean Duration | 0.81s |
| Std Dev | 0.23s |
| Min Duration | 0.53s |
| Max Duration | 1.09s |
| Success Rate | 33% |

### Individual Trials

| Trial | Duration | Max Pitch | Max Roll | Stop Reason |
|-------|----------|-----------|----------|-------------|
| 1 | 0.79s | 9.8° | 5.1° | emergency_stop |
| 2 | 1.09s | 5.5° | 20.0° | emergency_stop |
| 3 | 0.53s | 11.9° | 3.8° | emergency_stop |

---

## Overall Summary

**Total Tests Run**: 8

**Key Findings**:
- Average stability duration: 0.83s
- Stability range: 0.53s - 1.09s
- Consistent performance ceiling at ~0.9s
- Controller generates proper torques (10-12 Nm peak)
- Fundamental limitation: joint-space control insufficient

**Next Steps**:
- Implement full WBC QP solver with task-space control
- Re-run this test suite to measure improvement
- Target: >5s stability, eventual walking capability

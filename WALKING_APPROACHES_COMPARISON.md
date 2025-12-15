# Walking Implementation Approaches - Comparative Analysis

**Date**: 2025-11-25
**Purpose**: Compare different approaches to achieve robust dynamic walking

---

## Executive Summary

Three viable approaches to implement walking on the Hunter bipedal robot:

1. **Position Control + CoM Planning** (Your preferred approach) ⭐
2. **WBC with Increased Torque Limits** (Revisit Phase 1 findings)
3. **Hybrid: Position Control + WBC Force Distribution**

This document provides detailed comparison to help you make an informed decision.

---

## Approach 1: Position Control + CoM Planning ⭐

**See**: `PHASE_4_WALKING_PLAN.md` for detailed plan

### Architecture
```
Gait Planner → CoM Planner → Full-Body IK → Position Control
```

### Pros ✅
1. **Leverages proven stability**: Phase 2 position control (Roll=0.0°, Pitch=-1.8°)
2. **Avoids torque control issues**: No 20 Nm limit problem
3. **Novel approach**: Not commonly used, could be innovative contribution
4. **Simpler than WBC**: No QP optimization at runtime
5. **Robust to model uncertainty**: Position control naturally compensates

### Cons ❌
1. **Unproven for bipedal walking**: No reference implementation
2. **Full-body IK complexity**: Need to solve for base + legs simultaneously
3. **May be slow**: IK optimization at every timestep
4. **Limited dynamic capability**: Position control has lag
5. **Higher risk**: Novel approach, success not guaranteed

### Technical Challenges
1. **Full-Body IK Solver** (Phase 4.2):
   - Need nonlinear optimization (scipy.optimize.minimize)
   - Solve for 16 DOF: 6 base + 10 joints
   - Constraints: foot positions, CoM, orientation
   - May not converge for all configurations

2. **CoM Planning** (Phase 4.1):
   - Preview control or MPC
   - ZMP tracking to ensure stability
   - Requires understanding of inverted pendulum model

3. **Real-time Performance**:
   - IK optimization must run at 50-100 Hz
   - May need warm-starting and efficient implementation

### Estimated Timeline
- **Minimum success** (10 steps): 2 weeks
- **Target success** (60s walking): 3 weeks
- **Stretch success** (5min + disturbances): 4 weeks

### Success Probability
- **Minimum**: 80% (10+ steps)
- **Target**: 60% (60s walking)
- **Stretch**: 40% (disturbance rejection)

### When to Choose This
- You want to avoid torque control entirely
- You value innovation over proven methods
- You're willing to invest 3-4 weeks with moderate risk
- Disturbance rejection is high priority (feedback control easier with position control)

---

## Approach 2: WBC with Increased Torque Limits

### Architecture
```
MPC → CoM Trajectory → WBC QP → Inverse Dynamics → Torque Control
```

### Key Insight from Baseline Tests
Phase 1 tests showed:
- Required torques: 107-125 Nm (5-6x the 20 Nm limit)
- URDF torque capacity: **200 Nm** (10x current limit!)
- **Hypothesis**: 20 Nm limit was the problem, not torque control itself

### Pros ✅
1. **Industry standard**: Used in Honda ASIMO, Boston Dynamics robots
2. **Well-documented**: Extensive literature and references
3. **Leverages existing WBC**: Phase 1-2 already implemented WBC framework
4. **Optimal force distribution**: WBC computes optimal contact forces
5. **Quick validation**: Can test immediately (just increase torque limit)
6. **Flexible**: Can add complex constraints (friction cones, torque limits)

### Cons ❌
1. **Phase 1 failure**: Both controllers failed with torque control
2. **Torque saturation**: May still occur at higher limits
3. **Model dependency**: Requires accurate robot dynamics
4. **Computational cost**: QP optimization at 1 kHz
5. **Complexity**: Full WBC walking requires contact-aware optimization

### Technical Challenges
1. **Validate Torque Control** (1 day):
   ```bash
   WBC_TORQUE_LIMIT=100 WBC_TORQUE_CONTROL=1 \
     python3 src/main_simulation.py --mode wbc --duration 30
   ```
   - If this passes → proceed with WBC walking
   - If this fails → torque control fundamentally unsuitable

2. **Contact-Aware WBC** (Phase 4 equivalent):
   - Modify WBC to handle swing/stance phases
   - Add swing foot tracking task
   - Handle contact transitions smoothly

3. **MPC Integration**:
   - Current MPC only for standing
   - Extend to walking (CoM trajectory for swing phase)

### Estimated Timeline
- **Validation test**: 1 day (test with 100 Nm limit)
- **If validation passes**:
  - Minimum success: 1.5 weeks
  - Target success: 2.5 weeks
  - Stretch success: 3.5 weeks
- **If validation fails**: Approach abandoned (1 day wasted)

### Success Probability
- **Validation passes**: 70% (depends on torque limit being the only issue)
- **If validation passes**:
  - Minimum: 90% (proven approach)
  - Target: 75%
  - Stretch: 60%

### When to Choose This
- Quick validation test passes (WBC stable with 100 Nm limit)
- You want proven, industry-standard approach
- You value speed to implementation over innovation
- You're comfortable with torque control complexity

---

## Approach 3: Hybrid Position + WBC Force Distribution

### Architecture
```
MPC → CoM Trajectory → WBC (force planning) → Full-Body IK → Position Control
             ↓                                      ↑
      Desired Forces                         Joint Angles
```

**Key Idea**: Use WBC for **force planning** (not torque control), then use IK to convert forces to position commands.

### How It Works
1. **WBC computes optimal forces** (like current implementation)
2. **Don't apply forces as torques** (Phase 1 failed here)
3. **Instead**: Use forces to compute desired base motion
4. **Full-body IK** converts desired motion to joint angles
5. **Position control** executes joint angles (Phase 2 proven stable)

### Pros ✅
1. **Best of both worlds**: WBC optimality + position control stability
2. **Leverages existing code**: Both WBC and position control already implemented
3. **Proven components**: Combines two working systems
4. **Optimal force distribution**: WBC handles contact constraints
5. **Robust execution**: Position control stability from Phase 2

### Cons ❌
1. **Most complex**: Two major components (WBC + full-body IK)
2. **Force → motion mapping**: Non-trivial conversion
3. **Computational cost**: WBC QP + IK optimization
4. **Unproven integration**: Novel combination

### Technical Challenges
1. **Force to Motion Conversion**:
   ```python
   def force_to_desired_motion(contact_forces, current_state):
       # Compute base acceleration from forces
       # F = m * a  →  a = F / m
       base_accel = contact_forces / robot_mass

       # Integrate to get velocity and position
       base_vel = current_vel + base_accel * dt
       base_pos = current_pos + base_vel * dt

       return base_pos, base_vel
   ```

2. **Full-Body IK**: Same as Approach 1 (Phase 4.2)

3. **Synchronization**: WBC and IK must be consistent

### Estimated Timeline
- **Minimum success**: 3 weeks
- **Target success**: 4 weeks
- **Stretch success**: 5 weeks

### Success Probability
- **Minimum**: 70%
- **Target**: 50%
- **Stretch**: 35%

### When to Choose This
- Approach 1 (position control) seems too risky
- Approach 2 (WBC torque) validation fails
- You want optimal force distribution
- You have 4-5 weeks available

---

## Quick Comparison Table

| Aspect | Approach 1: Position + CoM | Approach 2: WBC Torque | Approach 3: Hybrid |
|--------|---------------------------|----------------------|-------------------|
| **Complexity** | Medium | High | Very High |
| **Timeline** | 3 weeks | 2.5 weeks* | 4 weeks |
| **Success Prob** | 60% | 75%* | 50% |
| **Risk** | Medium | Low* | High |
| **Innovation** | High | Low | Medium |
| **Robustness** | High | Medium | High |
| **Speed to 1st result** | 2 weeks | 1 day* | 3 weeks |
| **Proven approach?** | No | Yes | No |
| **Dependencies** | Full-body IK, CoM planning | Torque validation | Both Approach 1 & 2 |

\* Assumes torque validation passes (1-day test)

---

## Recommended Decision Process

### Step 1: Quick Validation (1 day)

Test WBC with increased torque limit:
```bash
# Test 1: MPCWBCController with 100 Nm limit
WBC_TORQUE_LIMIT=100 WBC_TORQUE_CONTROL=1 \
  python3 src/main_simulation.py --mode wbc --duration 30 --no-gui

# Test 2: WBCWalkingController with 100 Nm limit
WALKING_WBC=1 WBC_WALKING_STANDING=1 WBC_HYBRID_CONTROL=1 \
  WBC_TORQUE_LIMIT=100 \
  python3 src/main_simulation.py --mode walking --duration 30 --no-gui
```

### Step 2: Interpret Results

**If both tests PASS** ✅:
- **Recommendation**: Approach 2 (WBC with torque control)
- **Rationale**: Fastest path, proven method, high success probability
- **Timeline**: 2.5 weeks to target success

**If both tests FAIL** ❌:
- **Recommendation**: Approach 1 (Position + CoM planning)
- **Rationale**: Torque control fundamentally unsuitable, proven position control
- **Timeline**: 3 weeks to target success

**If results are mixed** ⚠️:
- **Option A**: Debug torque control issues (investigate why still failing)
- **Option B**: Proceed with Approach 1 (safer bet)

### Step 3: Commit to Approach

Once decided:
1. Create detailed phase plan (already done for Approach 1)
2. Create development branch
3. Start implementation
4. Regular progress checkpoints

---

## My Recommendation

### Primary Recommendation: **Start with Validation Test** (Approach 2)

**Reasoning**:
1. **Only 1 day investment** to test if torque control works with proper limits
2. **Low risk**: If it fails, proceed with Approach 1
3. **High reward**: If it passes, save 1-2 weeks using proven approach
4. **No wasted effort**: Validation data useful for any approach

**Action Plan**:
```bash
# Morning (2-3 hours): Run validation tests
WBC_TORQUE_LIMIT=100 WBC_TORQUE_CONTROL=1 \
  python3 src/main_simulation.py --mode wbc --duration 30 --no-gui

# Afternoon (2-3 hours): Analyze results and decide
# - If PASS: Create Phase 4 plan for WBC torque walking
# - If FAIL: Proceed with existing Phase 4 position control plan
```

### Secondary Recommendation: **Approach 1** (if validation fails)

Your preferred approach (Position Control + CoM Planning) is solid:
- Leverages Phase 2 proven stability
- Novel and innovative
- Good technical challenge
- 60% success probability for 60s walking

**Ready to proceed** if validation fails (plan already created).

### Not Recommended: **Approach 3** (unless both 1 & 2 fail)

Too complex, too risky, too long timeline. Only consider if:
- Approach 2 validation fails
- Approach 1 hits fundamental blocker (e.g., IK won't converge)

---

## Next Steps

### Immediate (Today)
1. **Review** PHASE_4_WALKING_PLAN.md and this comparison
2. **Decide** whether to run 1-day validation test first
3. **Approve** either:
   - Run validation → choose based on results
   - Skip validation → proceed with Approach 1

### This Week
- If validation: Complete tests, analyze, choose approach
- If Approach 1: Start Phase 4.1 (CoM planning)
- Document decision and rationale

### Weeks 2-3
- Execute chosen approach
- Regular progress updates
- Adjust timeline as needed

---

## Questions for You

1. **Validation Test**: Should we run the 1-day torque limit validation first?
   - **Pros**: Might unlock faster, proven path
   - **Cons**: 1 day delay if it fails
   - **My vote**: Yes, worth 1 day to potentially save 1 week

2. **Risk Tolerance**: What's your preference?
   - **Conservative**: Proven approach (Approach 2 if validation passes)
   - **Balanced**: Your preferred approach (Approach 1)
   - **Aggressive**: Novel hybrid (Approach 3)

3. **Timeline Flexibility**: What's your constraint?
   - **Tight** (2 weeks): Must choose Approach 2 if validation passes
   - **Moderate** (3 weeks): Approach 1 is feasible
   - **Flexible** (4+ weeks): Any approach viable

4. **Success Definition**: What's minimum acceptable outcome?
   - **Proof-of-concept**: 3-5 steps (achievable in 1-2 weeks with any approach)
   - **Functional walking**: 10+ steps, 30s duration (your current target)
   - **Production-ready**: 60s+ walking, disturbance rejection (stretch goal)

---

**Status**: 📋 Awaiting your decision

**Options**:
- **A**: Run 1-day validation test, then decide
- **B**: Skip validation, proceed with Approach 1 (Position + CoM)
- **C**: Skip validation, proceed with Approach 2 (WBC Torque, risky)
- **D**: Request modifications to plan

**My Recommendation**: **Option A** (validate first, then choose best path)

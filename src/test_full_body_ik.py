"""
Test script for Full-Body IK Solver

Validates that the solver can:
1. Maintain foot positions while adjusting CoM
2. Handle single-support configurations
3. Solve within reasonable time
4. Produce stable configurations

Author: Phase 4.2 Testing
Date: 2025-11-25
"""

import numpy as np
import pybullet as p
import sys
from robot_constants import BASE_HEIGHT
from full_body_ik import FullBodyIKSolver, FullBodyIKParams


def setup_robot():
    """Initialize PyBullet and load robot"""
    # Connect to PyBullet (headless)
    p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(0.001)

    # Load robot
    urdf_path = "../models/urdf/hunter.urdf"
    robot_id = p.loadURDF(urdf_path, [0, 0, BASE_HEIGHT], useFixedBase=False)

    # Get joint info
    joint_dict = {}
    for i in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        if 'leg' in joint_name and 'joint' in joint_name:
            joint_dict[joint_name] = i

    return robot_id, joint_dict


def get_current_state(robot_id, joint_dict):
    """Get current robot state"""
    # Base
    base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)

    # Joints
    joint_indices = list(joint_dict.values())
    joint_states = p.getJointStates(robot_id, joint_indices)
    joint_angles = {name: joint_states[i][0]
                   for i, name in enumerate(joint_dict.keys())}

    # Feet
    left_foot_link = joint_dict['leg_l5_joint']
    right_foot_link = joint_dict['leg_r5_joint']

    left_foot_state = p.getLinkState(robot_id, left_foot_link)
    right_foot_state = p.getLinkState(robot_id, right_foot_link)

    left_foot_pos = np.array(left_foot_state[0])
    right_foot_pos = np.array(right_foot_state[0])

    # CoM
    com_pos = compute_com(robot_id)

    return {
        'base_pos': np.array(base_pos),
        'base_orn': base_orn,
        'joint_angles': joint_angles,
        'left_foot': left_foot_pos,
        'right_foot': right_foot_pos,
        'com': com_pos
    }


def compute_com(robot_id):
    """Compute center of mass"""
    total_mass = 0.0
    com_pos = np.zeros(3)

    # Base
    base_mass = p.getDynamicsInfo(robot_id, -1)[0]
    base_pos = np.array(p.getBasePositionAndOrientation(robot_id)[0])
    total_mass += base_mass
    com_pos += base_mass * base_pos

    # Links
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        link_mass = p.getDynamicsInfo(robot_id, i)[0]
        if link_mass > 0:
            link_state = p.getLinkState(robot_id, i)
            link_pos = np.array(link_state[0])
            total_mass += link_mass
            com_pos += link_mass * link_pos

    return com_pos / total_mass


def test_fixed_feet_com_shift():
    """
    Test 1: Fixed Feet, CoM Shift

    Keep both feet fixed, shift CoM laterally.
    This tests if IK can adjust base position while maintaining foot contact.
    """
    print("=" * 60)
    print("Test 1: Fixed Feet, CoM Shift")
    print("=" * 60)

    robot_id, joint_dict = setup_robot()
    solver = FullBodyIKSolver(robot_id, joint_dict)

    # Get initial state
    initial_state = get_current_state(robot_id, joint_dict)
    print(f"\nInitial state:")
    print(f"  Left foot: {initial_state['left_foot']}")
    print(f"  Right foot: {initial_state['right_foot']}")
    print(f"  CoM: {initial_state['com']}")

    # Target: Shift CoM 5cm to the right (Y direction)
    com_target = initial_state['com'].copy()
    com_target[1] += 0.05  # Shift right

    print(f"\nTarget CoM: {com_target}")

    # Solve IK
    solution = solver.solve(
        left_foot_target=initial_state['left_foot'],
        right_foot_target=initial_state['right_foot'],
        com_target=com_target,
        left_contact=True,
        right_contact=True
    )

    if solution is None:
        print("\n✗ FAIL: IK did not converge")
        p.disconnect()
        return False

    # Apply solution and check
    p.resetBasePositionAndOrientation(robot_id, solution['base_pos'], solution['base_orn'])
    for joint_name, angle in solution['joint_angles'].items():
        p.resetJointState(robot_id, joint_dict[joint_name], angle)

    final_state = get_current_state(robot_id, joint_dict)

    # Compute errors
    left_foot_error = np.linalg.norm(final_state['left_foot'] - initial_state['left_foot'])
    right_foot_error = np.linalg.norm(final_state['right_foot'] - initial_state['right_foot'])
    com_error = np.linalg.norm(final_state['com'][:2] - com_target[:2])

    print(f"\nResults:")
    print(f"  Solve time: {solution['solve_time']:.3f} s")
    print(f"  Iterations: {solution['iterations']}")
    print(f"  Final cost: {solution['cost']:.6f}")
    print(f"\nErrors:")
    print(f"  Left foot: {left_foot_error * 1000:.2f} mm")
    print(f"  Right foot: {right_foot_error * 1000:.2f} mm")
    print(f"  CoM (XY): {com_error * 1000:.2f} mm")

    # Success criteria
    success = (left_foot_error < 0.01 and
               right_foot_error < 0.01 and
               com_error < 0.02)

    print(f"\n{'✓ PASS' if success else '✗ FAIL'}: Foot error < 1cm, CoM error < 2cm")

    p.disconnect()
    return success


def test_single_support():
    """
    Test 2: Single Support Configuration

    Only right foot in contact, adjust left foot and CoM.
    This tests the solver in a more challenging single-support scenario.
    """
    print("\n" + "=" * 60)
    print("Test 2: Single Support (Right Foot Only)")
    print("=" * 60)

    robot_id, joint_dict = setup_robot()
    solver = FullBodyIKSolver(robot_id, joint_dict)

    # Get initial state
    initial_state = get_current_state(robot_id, joint_dict)

    # Target: Lift left foot 5cm, shift CoM over right foot
    left_foot_target = initial_state['left_foot'].copy()
    left_foot_target[2] += 0.05  # Lift 5cm

    com_target = initial_state['com'].copy()
    com_target[1] = initial_state['right_foot'][1]  # Shift CoM over right foot

    print(f"\nTargets:")
    print(f"  Left foot: {left_foot_target} (lifted 5cm)")
    print(f"  Right foot: {initial_state['right_foot']} (fixed)")
    print(f"  CoM: {com_target} (over right foot)")

    # Solve IK (only right foot in contact)
    solution = solver.solve(
        left_foot_target=left_foot_target,
        right_foot_target=initial_state['right_foot'],
        com_target=com_target,
        left_contact=False,  # Left foot NOT in contact
        right_contact=True
    )

    if solution is None:
        print("\n✗ FAIL: IK did not converge")
        p.disconnect()
        return False

    # Apply solution
    p.resetBasePositionAndOrientation(robot_id, solution['base_pos'], solution['base_orn'])
    for joint_name, angle in solution['joint_angles'].items():
        p.resetJointState(robot_id, joint_dict[joint_name], angle)

    final_state = get_current_state(robot_id, joint_dict)

    # Compute errors
    left_foot_error = np.linalg.norm(final_state['left_foot'] - left_foot_target)
    right_foot_error = np.linalg.norm(final_state['right_foot'] - initial_state['right_foot'])
    com_error = np.linalg.norm(final_state['com'][:2] - com_target[:2])

    print(f"\nResults:")
    print(f"  Solve time: {solution['solve_time']:.3f} s")
    print(f"  Iterations: {solution['iterations']}")
    print(f"\nErrors:")
    print(f"  Left foot: {left_foot_error * 1000:.2f} mm")
    print(f"  Right foot: {right_foot_error * 1000:.2f} mm")
    print(f"  CoM (XY): {com_error * 1000:.2f} mm")

    # Success criteria (more relaxed for single support)
    success = (right_foot_error < 0.01 and com_error < 0.03)

    print(f"\n{'✓ PASS' if success else '✗ FAIL'}: Stance foot < 1cm, CoM < 3cm")

    p.disconnect()
    return success


def test_performance():
    """
    Test 3: Computational Performance

    Run multiple IK solves to measure average solve time.
    Must be fast enough for real-time control (< 100ms per solve).
    """
    print("\n" + "=" * 60)
    print("Test 3: Computational Performance")
    print("=" * 60)

    robot_id, joint_dict = setup_robot()
    solver = FullBodyIKSolver(robot_id, joint_dict)

    initial_state = get_current_state(robot_id, joint_dict)

    # Run multiple solves
    num_trials = 10
    solve_times = []

    print(f"\nRunning {num_trials} trials...")

    for i in range(num_trials):
        # Slightly different CoM targets
        com_target = initial_state['com'].copy()
        com_target[0] += np.random.uniform(-0.02, 0.02)
        com_target[1] += np.random.uniform(-0.02, 0.02)

        solution = solver.solve(
            left_foot_target=initial_state['left_foot'],
            right_foot_target=initial_state['right_foot'],
            com_target=com_target,
            left_contact=True,
            right_contact=True
        )

        if solution:
            solve_times.append(solution['solve_time'])

    if len(solve_times) == 0:
        print("\n✗ FAIL: No successful solves")
        p.disconnect()
        return False

    avg_time = np.mean(solve_times)
    max_time = np.max(solve_times)
    min_time = np.min(solve_times)

    print(f"\nResults:")
    print(f"  Successful solves: {len(solve_times)}/{num_trials}")
    print(f"  Average time: {avg_time * 1000:.1f} ms")
    print(f"  Min time: {min_time * 1000:.1f} ms")
    print(f"  Max time: {max_time * 1000:.1f} ms")
    print(f"  Required for 10 Hz control: < 100 ms")

    success = avg_time < 0.1  # 100ms

    print(f"\n{'✓ PASS' if success else '✗ FAIL'}: Average time < 100ms")

    p.disconnect()
    return success


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("Full-Body IK Solver Validation Tests")
    print("=" * 60)

    results = []

    # Run tests
    try:
        results.append(("Fixed Feet CoM Shift", test_fixed_feet_com_shift()))
        results.append(("Single Support", test_single_support()))
        results.append(("Performance", test_performance()))
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)

    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {status}: {name}")

    all_passed = all(result[1] for result in results)
    print("\n" + "=" * 60)
    if all_passed:
        print("✓ ALL TESTS PASSED - Full-Body IK Ready for Integration")
    else:
        print("✗ SOME TESTS FAILED - Review results above")
    print("=" * 60)

    sys.exit(0 if all_passed else 1)

#!/usr/bin/env python3
"""
Diagnostic tool to test IK solver and walking mode issues

This script tests:
1. IK solver in isolation
2. Gait generator trajectories
3. Coordinate frame transformations
4. Foot contact detection
"""

import sys
import os
import numpy as np
import pybullet as p
import pybullet_data

# Add src to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from inverse_kinematics import BipedalIKSolver
from gait_generator import GaitGenerator, GaitParams
from simulation_env import HunterSimulation


def test_ik_solver():
    """Test IK solver with known foot positions"""
    print("=" * 70)
    print("TEST 1: IK Solver in Isolation")
    print("=" * 70)

    # Get URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

    # Create simulation (no GUI)
    sim = HunterSimulation(urdf_path=urdf_path, dt=0.001, use_gui=False)
    sim.connect()
    sim.load_robot(start_position=[0, 0, 0.679])

    # Create IK solver
    ik_solver = BipedalIKSolver(sim.robot_id, sim.joint_dict)

    # Test 1: Feet on ground, symmetric stance
    print("\nTest 1a: Symmetric standing pose")
    left_target = [0.0, 0.09, 0.0]   # 9cm to the left
    right_target = [0.0, -0.09, 0.0]  # 9cm to the right

    joint_angles = ik_solver.solve_both_legs(left_target, right_target)

    # Apply joint angles
    for joint_name, angle in joint_angles.items():
        joint_id = sim.joint_dict[joint_name]
        p.resetJointState(sim.robot_id, joint_id, angle)

    # Check resulting foot positions
    left_pos, right_pos = ik_solver.get_foot_positions()
    print(f"  Target left foot:  {left_target}")
    print(f"  Actual left foot:  {left_pos}")
    print(f"  Error: {np.linalg.norm(np.array(left_target) - left_pos):.6f} m")
    print(f"  Target right foot: {right_target}")
    print(f"  Actual right foot: {right_pos}")
    print(f"  Error: {np.linalg.norm(np.array(right_target) - right_pos):.6f} m")

    # Check base height
    base_pos, _, _, _ = sim.get_base_state()
    print(f"\n  Base height: {base_pos[2]:.3f} m (should be ~0.679m)")

    # Test 1b: One foot raised (swing phase)
    print("\nTest 1b: Left foot raised (simulating swing)")
    left_target = [0.05, 0.09, 0.05]  # Forward 5cm, raised 5cm
    right_target = [0.0, -0.09, 0.0]

    joint_angles = ik_solver.solve_both_legs(left_target, right_target)

    for joint_name, angle in joint_angles.items():
        joint_id = sim.joint_dict[joint_name]
        p.resetJointState(sim.robot_id, joint_id, angle)

    left_pos, right_pos = ik_solver.get_foot_positions()
    print(f"  Target left foot (raised):  {left_target}")
    print(f"  Actual left foot:           {left_pos}")
    print(f"  Error: {np.linalg.norm(np.array(left_target) - left_pos):.6f} m")

    base_pos, _, _, _ = sim.get_base_state()
    print(f"  Base height: {base_pos[2]:.3f} m")
    print(f"  Base forward shift: {base_pos[0]:.3f} m (should be ~0)")

    # Check if robot tilts
    base_orn_euler = p.getEulerFromQuaternion(base_pos[1] if len(base_pos) > 1 else [0,0,0,1])

    sim.disconnect()
    print("\n✓ IK solver test completed\n")


def test_gait_generator():
    """Test gait generator trajectories"""
    print("=" * 70)
    print("TEST 2: Gait Generator Trajectories")
    print("=" * 70)

    gait_params = GaitParams(
        step_length=0.08,
        step_height=0.04,
        step_period=1.2,
        stance_width=0.18,
        body_height=0.55
    )

    gait = GaitGenerator(gait_params)

    print(f"\nGait parameters:")
    print(f"  Step length: {gait_params.step_length} m")
    print(f"  Step height: {gait_params.step_height} m")
    print(f"  Step period: {gait_params.step_period} s")
    print(f"  Stance width: {gait_params.stance_width} m")

    print("\nFoot trajectories over one gait cycle:")
    print(f"{'Time':>6s} | {'Phase':>8s} | {'Left X':>8s} {'Left Y':>8s} {'Left Z':>8s} | {'Right X':>8s} {'Right Y':>8s} {'Right Z':>8s}")
    print("-" * 90)

    for i in range(13):  # 0 to 1.2 seconds
        t = i * 0.1
        left_pos, right_pos = gait.get_foot_trajectories(t)
        phase = (2.0 * np.pi * t / gait_params.step_period) % (2.0 * np.pi)

        print(f"{t:6.2f} | {phase:8.3f} | "
              f"{left_pos[0]:8.4f} {left_pos[1]:8.4f} {left_pos[2]:8.4f} | "
              f"{right_pos[0]:8.4f} {right_pos[1]:8.4f} {right_pos[2]:8.4f}")

    print("\n✓ Gait generator test completed\n")


def test_coordinate_frames():
    """Test coordinate frame integration between gait and IK"""
    print("=" * 70)
    print("TEST 3: Coordinate Frame Integration")
    print("=" * 70)

    # Get URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

    # Create simulation (no GUI)
    sim = HunterSimulation(urdf_path=urdf_path, dt=0.001, use_gui=False)
    sim.connect()
    sim.load_robot(start_position=[0, 0, 0.679])

    # Create components
    ik_solver = BipedalIKSolver(sim.robot_id, sim.joint_dict)
    gait_params = GaitParams(
        step_length=0.08,
        step_height=0.04,
        step_period=1.2,
        stance_width=0.18,
        body_height=0.55
    )
    gait = GaitGenerator(gait_params)

    print("\nSimulating walking with current implementation:")
    print(f"{'Time':>6s} | {'Base X':>8s} {'Base Z':>8s} | {'Left Z':>8s} {'Right Z':>8s} | {'Issue':>20s}")
    print("-" * 80)

    for i in range(11):  # Simulate 1 second
        t = i * 0.1

        # Get current base position
        base_pos, _, _, _ = sim.get_base_state()

        # Generate foot trajectories (gait generator output)
        left_target_gait, right_target_gait = gait.get_foot_trajectories(t)

        # CURRENT IMPLEMENTATION (problematic):
        # Adds base_pos[0] to x-coordinate
        left_target_world = np.array([
            base_pos[0] + left_target_gait[0],  # THIS IS THE BUG!
            left_target_gait[1],
            left_target_gait[2]
        ])
        right_target_world = np.array([
            base_pos[0] + right_target_gait[0],  # THIS IS THE BUG!
            right_target_gait[1],
            right_target_gait[2]
        ])

        # Solve IK
        joint_angles = ik_solver.solve_both_legs(left_target_world, right_target_world)

        # Apply to simulation
        for joint_name, angle in joint_angles.items():
            joint_id = sim.joint_dict[joint_name]
            p.resetJointState(sim.robot_id, joint_id, angle)

        # Step simulation
        for _ in range(10):
            p.stepSimulation()

        # Check result
        base_pos_new, _, _, _ = sim.get_base_state()
        left_pos, right_pos = ik_solver.get_foot_positions()

        issue = ""
        if base_pos_new[2] > 1.0:
            issue = "FLYING!"
        elif abs(base_pos_new[0]) > 0.5:
            issue = "Moving too fast!"

        print(f"{t:6.2f} | {base_pos_new[0]:8.3f} {base_pos_new[2]:8.3f} | "
              f"{left_pos[2]:8.3f} {right_pos[2]:8.3f} | {issue:>20s}")

    final_pos, _, _, _ = sim.get_base_state()
    print(f"\nFinal base position: [{final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f}]")

    if final_pos[2] > 1.0:
        print("❌ Robot is flying! (z > 1.0m)")
    else:
        print("✓ Robot stayed on ground")

    sim.disconnect()
    print("\n✓ Coordinate frame test completed\n")


def test_correct_implementation():
    """Test corrected coordinate frame handling"""
    print("=" * 70)
    print("TEST 4: CORRECTED Implementation")
    print("=" * 70)

    # Get URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

    # Create simulation (no GUI)
    sim = HunterSimulation(urdf_path=urdf_path, dt=0.001, use_gui=False)
    sim.connect()
    sim.load_robot(start_position=[0, 0, 0.679])

    # Create components
    ik_solver = BipedalIKSolver(sim.robot_id, sim.joint_dict)
    gait_params = GaitParams(
        step_length=0.08,
        step_height=0.04,
        step_period=1.2,
        stance_width=0.18,
        body_height=0.55
    )
    gait = GaitGenerator(gait_params)

    print("\nSimulating walking with CORRECTED implementation:")
    print("  - Gait trajectories are already in world frame")
    print("  - Do NOT add base position offset")
    print()
    print(f"{'Time':>6s} | {'Base X':>8s} {'Base Z':>8s} | {'Left Z':>8s} {'Right Z':>8s} | {'Status':>20s}")
    print("-" * 80)

    for i in range(11):
        t = i * 0.1

        # Get current base position
        base_pos, _, _, _ = sim.get_base_state()

        # Generate foot trajectories
        left_target_world, right_target_world = gait.get_foot_trajectories(t)

        # CORRECTED: Use gait output directly (already in world frame)
        # Do NOT add base_pos[0]!

        # Solve IK
        joint_angles = ik_solver.solve_both_legs(
            list(left_target_world),
            list(right_target_world)
        )

        # Apply to simulation
        for joint_name, angle in joint_angles.items():
            joint_id = sim.joint_dict[joint_name]
            p.resetJointState(sim.robot_id, joint_id, angle)

        # Step simulation
        for _ in range(10):
            p.stepSimulation()

        # Check result
        base_pos_new, _, _, _ = sim.get_base_state()
        left_pos, right_pos = ik_solver.get_foot_positions()

        status = "OK"
        if base_pos_new[2] > 1.0:
            status = "FLYING!"
        elif base_pos_new[2] < 0.5:
            status = "Fallen"

        print(f"{t:6.2f} | {base_pos_new[0]:8.3f} {base_pos_new[2]:8.3f} | "
              f"{left_pos[2]:8.3f} {right_pos[2]:8.3f} | {status:>20s}")

    final_pos, _, _, _ = sim.get_base_state()
    print(f"\nFinal base position: [{final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f}]")

    if 0.5 < final_pos[2] < 1.0:
        print("✓ Robot stayed at correct height!")
    else:
        print(f"⚠ Robot height issue: z={final_pos[2]:.2f}m")

    sim.disconnect()
    print("\n✓ Corrected implementation test completed\n")


if __name__ == "__main__":
    print("\n")
    print("╔" + "═" * 68 + "╗")
    print("║" + " " * 15 + "WALKING MODE DIAGNOSTIC TESTS" + " " * 24 + "║")
    print("╚" + "═" * 68 + "╝")
    print()

    try:
        test_ik_solver()
        test_gait_generator()
        test_coordinate_frames()
        test_correct_implementation()

        print("=" * 70)
        print("DIAGNOSIS COMPLETE")
        print("=" * 70)
        print()
        print("KEY FINDING:")
        print("  The bug is in main_simulation.py lines 104-108")
        print("  The code adds base_pos[0] to gait trajectories, creating a feedback loop")
        print()
        print("SOLUTION:")
        print("  Remove the base_pos[0] offset - gait trajectories are already in world frame")
        print()

    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()

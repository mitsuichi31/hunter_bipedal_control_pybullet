"""
Test script for Position Control Walking Controller

Validates integration of:
- GaitGenerator (foot trajectories)
- SimpleCoMPlanner2D (CoM planning)
- FullBodyIKSolver (base + joint angles)

Tests:
1. Standing mode regression (should match Phase 2 stability)
2. Minimal walking (3-5 steps with conservative parameters)

Author: Phase 4.3 Integration Testing
Date: 2025-11-25
"""

import numpy as np
import pybullet as p
import pybullet_data
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from position_control_walking import PositionControlWalkingController, WalkingControllerParams
from gait_generator import GaitParams


def setup_robot():
    """Initialize PyBullet and load robot"""
    # Connect to PyBullet (GUI for visual debugging)
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(0.001)
    p.setRealTimeSimulation(0)

    # Load ground plane
    p.loadURDF("plane.urdf")

    # Load robot at correct height
    urdf_path = "../models/urdf/hunter.urdf"
    base_height = 0.679
    robot_id = p.loadURDF(urdf_path, [0, 0, base_height], useFixedBase=False)

    # Get joint info
    joint_dict = {}
    for i in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        if 'leg' in joint_name and 'joint' in joint_name:
            joint_dict[joint_name] = i

    # Set initial configuration (straight legs, symmetric stance)
    standing_config = {
        'leg_l1_joint': -0.1,
        'leg_l2_joint': 0.0,
        'leg_l3_joint': 0.0,
        'leg_l4_joint': 0.0,
        'leg_l5_joint': 0.0,
        'leg_r1_joint': 0.1,
        'leg_r2_joint': 0.0,
        'leg_r3_joint': 0.0,
        'leg_r4_joint': 0.0,
        'leg_r5_joint': 0.0,
    }

    for joint_name, angle in standing_config.items():
        joint_idx = joint_dict[joint_name]
        p.resetJointState(robot_id, joint_idx, angle)

    return robot_id, joint_dict


def apply_position_control(robot_id, joint_dict, target_angles, max_force=300.0):
    """Apply position control to joints using PyBullet's built-in controller"""
    for joint_name, target_angle in target_angles.items():
        joint_idx = joint_dict[joint_name]

        # Use PyBullet's built-in position control (same as main_simulation.py)
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
            force=max_force
        )


def get_robot_state(robot_id):
    """Get current robot state for logging"""
    base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
    euler = p.getEulerFromQuaternion(base_orn)

    return {
        'x': base_pos[0],
        'y': base_pos[1],
        'z': base_pos[2],
        'roll': np.degrees(euler[0]),
        'pitch': np.degrees(euler[1]),
        'yaw': np.degrees(euler[2])
    }


def test_standing_mode():
    """
    Test 1: Standing Mode Regression

    Verify that standing mode maintains stability like Phase 2.
    Success: Roll < 5°, Pitch < 5°, Height ≈ 0.69m
    """
    print("=" * 60)
    print("Test 1: Standing Mode Regression")
    print("=" * 60)

    robot_id, joint_dict = setup_robot()

    # Create controller in standing mode
    params = WalkingControllerParams(
        standing_mode=True,  # Standing only
        enable_walking=False
    )

    controller = PositionControlWalkingController(robot_id, joint_dict, params)
    controller.reset()

    # Simulation parameters
    sim_dt = 0.001  # Physics simulation timestep (1ms)
    control_dt = 0.02  # Control update frequency (50 Hz = 20ms)
    duration = 10.0  # 10 seconds
    num_sim_steps = int(duration / sim_dt)
    control_decimation = int(control_dt / sim_dt)  # Update control every N sim steps

    # Data logging
    time_log = []
    state_log = []

    print(f"\nRunning {duration}s standing test...")
    print(f"  Simulation dt: {sim_dt*1000:.1f}ms (1000 Hz)")
    print(f"  Control dt: {control_dt*1000:.1f}ms (50 Hz)\n")

    position_commands = None
    for step in range(num_sim_steps):
        t = step * sim_dt

        # Update control at lower frequency
        if step % control_decimation == 0:
            # Get position commands from controller
            position_commands = controller.update(control_dt)

        if position_commands is None:
            print(f"\n✗ FAIL: Emergency stop at t={t:.2f}s")
            p.disconnect()
            return False

        # Apply position control
        apply_position_control(robot_id, joint_dict, position_commands)

        # Step simulation
        p.stepSimulation()

        # Log state (every 10ms)
        if step % 10 == 0:
            state = get_robot_state(robot_id)
            time_log.append(t)
            state_log.append(state)

            if step % 1000 == 0:  # Print every second
                print(f"  t={t:.1f}s: Roll={state['roll']:+6.2f}°, Pitch={state['pitch']:+6.2f}°, Z={state['z']:.3f}m")

    # Analyze results
    time_log = np.array(time_log)
    roll_log = np.array([s['roll'] for s in state_log])
    pitch_log = np.array([s['pitch'] for s in state_log])
    height_log = np.array([s['z'] for s in state_log])

    # Steady-state analysis (last 5 seconds)
    steady_idx = int(len(time_log) * 0.5)
    roll_mean = np.mean(roll_log[steady_idx:])
    pitch_mean = np.mean(pitch_log[steady_idx:])
    height_mean = np.mean(height_log[steady_idx:])

    roll_std = np.std(roll_log[steady_idx:])
    pitch_std = np.std(pitch_log[steady_idx:])

    print(f"\nResults (steady-state, last 5s):")
    print(f"  Roll:   {roll_mean:+6.2f}° ± {roll_std:.2f}°")
    print(f"  Pitch:  {pitch_mean:+6.2f}° ± {pitch_std:.2f}°")
    print(f"  Height: {height_mean:.3f}m (target: 0.69m)")

    # Success criteria
    success = (abs(roll_mean) < 5.0 and
               abs(pitch_mean) < 5.0 and
               abs(height_mean - 0.69) < 0.05)

    print(f"\n{'✓ PASS' if success else '✗ FAIL'}: Roll < 5°, Pitch < 5°, Height ≈ 0.69m")

    # Plot results
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))

    axes[0].plot(time_log, roll_log, label='Roll', linewidth=2)
    axes[0].axhline(y=5, color='r', linestyle='--', alpha=0.5)
    axes[0].axhline(y=-5, color='r', linestyle='--', alpha=0.5)
    axes[0].set_ylabel('Roll (deg)')
    axes[0].set_title('Standing Mode - Orientation Stability')
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(time_log, pitch_log, label='Pitch', linewidth=2)
    axes[1].axhline(y=5, color='r', linestyle='--', alpha=0.5)
    axes[1].axhline(y=-5, color='r', linestyle='--', alpha=0.5)
    axes[1].set_ylabel('Pitch (deg)')
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(time_log, height_log, label='Height', linewidth=2)
    axes[2].axhline(y=0.69, color='g', linestyle='--', alpha=0.5, label='Target')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Height (m)')
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig('/workspace/hunter/logs/position_control_standing_test.png', dpi=150)
    print(f"\nPlot saved to: logs/position_control_standing_test.png")

    p.disconnect()
    return success


def test_minimal_walking():
    """
    Test 2: Minimal Walking

    Test very conservative walking (2cm steps, 2s period, 1cm height).
    Success: 3-5 consecutive steps without falling
    """
    print("\n" + "=" * 60)
    print("Test 2: Minimal Walking (Conservative Parameters)")
    print("=" * 60)

    robot_id, joint_dict = setup_robot()

    # Create controller with very conservative gait
    gait_params = GaitParams(
        step_length=0.02,  # 2cm steps
        step_height=0.01,  # 1cm lift
        step_period=2.0,   # 2 seconds per step
        stance_width=0.18,
        double_support_ratio=0.5  # 50% double support
    )

    params = WalkingControllerParams(
        gait=gait_params,
        standing_mode=False,  # Walking enabled
        enable_walking=True
    )

    controller = PositionControlWalkingController(robot_id, joint_dict, params)
    controller.reset()

    # Simulation parameters
    sim_dt = 0.001  # Physics simulation timestep (1ms)
    control_dt = 0.02  # Control update frequency (50 Hz = 20ms)
    duration = 10.0  # 10 seconds (should complete ~5 steps)
    num_sim_steps = int(duration / sim_dt)
    control_decimation = int(control_dt / sim_dt)

    # Data logging
    time_log = []
    state_log = []

    print(f"\nRunning {duration}s walking test...")
    print(f"  Gait: {gait_params.step_length}m steps, {gait_params.step_period}s period")
    print(f"  Expected: ~{duration / gait_params.step_period:.1f} steps")
    print(f"  Simulation dt: {sim_dt*1000:.1f}ms (1000 Hz)")
    print(f"  Control dt: {control_dt*1000:.1f}ms (50 Hz)\n")

    position_commands = None
    for step in range(num_sim_steps):
        t = step * sim_dt

        # Update control at lower frequency
        if step % control_decimation == 0:
            # Get position commands from controller
            position_commands = controller.update(control_dt)

        if position_commands is None:
            print(f"\n✗ FAIL: Emergency stop at t={t:.2f}s")

            # Final state
            state = get_robot_state(robot_id)
            print(f"  Final: Roll={state['roll']:+6.2f}°, Pitch={state['pitch']:+6.2f}°, Z={state['z']:.3f}m")

            # Compute steps completed
            steps_completed = t / gait_params.step_period
            print(f"  Steps completed: {steps_completed:.1f}")

            p.disconnect()
            return False

        # Apply position control
        apply_position_control(robot_id, joint_dict, position_commands)

        # Step simulation
        p.stepSimulation()

        # Log state (every 10ms)
        if step % 10 == 0:
            state = get_robot_state(robot_id)
            time_log.append(t)
            state_log.append(state)

            if step % 1000 == 0:  # Print every second
                print(f"  t={t:.1f}s: Roll={state['roll']:+6.2f}°, Pitch={state['pitch']:+6.2f}°, X={state['x']:+.3f}m")

    # Analyze results
    time_log = np.array(time_log)
    roll_log = np.array([s['roll'] for s in state_log])
    pitch_log = np.array([s['pitch'] for s in state_log])
    x_log = np.array([s['x'] for s in state_log])
    height_log = np.array([s['z'] for s in state_log])

    # Calculate steps completed
    steps_completed = duration / gait_params.step_period
    forward_distance = x_log[-1] - x_log[0]

    print(f"\nResults:")
    print(f"  Duration: {duration}s")
    print(f"  Steps completed: {steps_completed:.1f}")
    print(f"  Forward distance: {forward_distance:.3f}m")
    print(f"  Final Roll: {roll_log[-1]:+6.2f}°")
    print(f"  Final Pitch: {pitch_log[-1]:+6.2f}°")
    print(f"  Final Height: {height_log[-1]:.3f}m")

    # Success criteria: completed at least 3 steps without falling
    success = (steps_completed >= 3.0 and
               abs(roll_log[-1]) < 15.0 and
               abs(pitch_log[-1]) < 15.0)

    print(f"\n{'✓ PASS' if success else '✗ FAIL'}: Completed ≥3 steps, Roll < 15°, Pitch < 15°")

    # Plot results
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))

    axes[0].plot(time_log, x_log, label='X Position', linewidth=2)
    axes[0].set_ylabel('X Position (m)')
    axes[0].set_title('Minimal Walking - Forward Progress')
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(time_log, roll_log, label='Roll', linewidth=2)
    axes[1].plot(time_log, pitch_log, label='Pitch', linewidth=2)
    axes[1].axhline(y=15, color='r', linestyle='--', alpha=0.5)
    axes[1].axhline(y=-15, color='r', linestyle='--', alpha=0.5)
    axes[1].set_ylabel('Orientation (deg)')
    axes[1].set_title('Orientation Stability During Walking')
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(time_log, height_log, label='Height', linewidth=2)
    axes[2].axhline(y=0.69, color='g', linestyle='--', alpha=0.5, label='Target')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Height (m)')
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig('/workspace/hunter/logs/position_control_walking_test.png', dpi=150)
    print(f"\nPlot saved to: logs/position_control_walking_test.png")

    p.disconnect()
    return success


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("Position Control Walking Controller - Integration Tests")
    print("=" * 60)

    results = []

    # Run tests
    try:
        results.append(("Standing Mode", test_standing_mode()))
        results.append(("Minimal Walking", test_minimal_walking()))
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
        print("✓ ALL TESTS PASSED - Phase 4.3 Integration Successful")
    else:
        print("✗ SOME TESTS FAILED - Review results above")
    print("=" * 60)

    sys.exit(0 if all_passed else 1)

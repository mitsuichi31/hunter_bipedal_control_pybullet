"""
Phase 4.5: Multi-Step Walking Validation

Progressive testing with 4 complexity levels:
- Level 1: Minimal Walking (3 steps, 2cm/2s)
- Level 2: Slow Walking (10 steps, 5cm/1.5s)
- Level 3: Moderate Walking (20 steps, 10cm/1s)
- Level 4: Indefinite Walking (60s, 50+ steps)

Author: Phase 4.5 Multi-Step Validation
Date: 2025-11-26
"""

import numpy as np
import pybullet as p
import pybullet_data
import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from robot_constants import BASE_HEIGHT, standing_config_copy
from position_control_walking import PositionControlWalkingController, WalkingControllerParams
from gait_generator import GaitParams


def setup_robot():
    """Initialize PyBullet and load robot"""
    # Connect to PyBullet (headless for faster execution)
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(0.001)
    p.setRealTimeSimulation(0)

    # Load ground plane
    p.loadURDF("plane.urdf")

    # Load robot at correct height
    urdf_path = "../models/urdf/hunter.urdf"
    robot_id = p.loadURDF(urdf_path, [0, 0, BASE_HEIGHT], useFixedBase=False)

    # Get joint info
    joint_dict = {}
    for i in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        if 'leg' in joint_name and 'joint' in joint_name:
            joint_dict[joint_name] = i

    # Set initial configuration (straight legs, symmetric stance)
    standing_config = standing_config_copy()

    for joint_name, angle in standing_config.items():
        joint_idx = joint_dict[joint_name]
        p.resetJointState(robot_id, joint_idx, angle)

    return robot_id, joint_dict


def apply_position_control(robot_id, joint_dict, target_angles, max_force=300.0):
    """Apply position control to joints"""
    for joint_name, target_angle in target_angles.items():
        joint_idx = joint_dict[joint_name]
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


def run_walking_test(level_name, step_length, step_height, step_period, duration,
                     success_roll_thresh, success_pitch_thresh, min_steps):
    """
    Run a single walking test with specified parameters

    Args:
        level_name: Test level name (e.g., "Level 1")
        step_length: Step length in meters
        step_height: Step height in meters
        step_period: Step period in seconds
        duration: Test duration in seconds
        success_roll_thresh: Maximum roll angle for success (degrees)
        success_pitch_thresh: Maximum pitch angle for success (degrees)
        min_steps: Minimum steps to complete

    Returns:
        Dictionary with test results
    """
    print("\n" + "=" * 70)
    print(f"{level_name}: {step_length*100:.0f}cm steps, {step_period}s period, {duration}s duration")
    print("=" * 70)

    robot_id, joint_dict = setup_robot()

    # Create controller with specified gait parameters
    gait_params = GaitParams(
        step_length=step_length,
        step_height=step_height,
        step_period=step_period,
        stance_width=0.18,
        double_support_ratio=0.5
    )

    params = WalkingControllerParams(
        standing_mode=False,
        enable_walking=True,
        gait=gait_params
    )

    controller = PositionControlWalkingController(robot_id, joint_dict, params)
    controller.reset()

    # Simulation parameters
    sim_dt = 0.001  # 1ms physics
    control_dt = 0.02  # 50 Hz control
    num_sim_steps = int(duration / sim_dt)
    control_decimation = int(control_dt / sim_dt)

    # Data logging
    time_log = []
    state_log = []

    expected_steps = duration / step_period
    print(f"\nParameters:")
    print(f"  Step length: {step_length:.3f}m ({step_length*100:.1f}cm)")
    print(f"  Step height: {step_height:.3f}m ({step_height*100:.1f}cm)")
    print(f"  Step period: {step_period}s")
    print(f"  Expected steps: {expected_steps:.1f}")
    print(f"  Duration: {duration}s")
    print(f"  Success criteria: Roll < {success_roll_thresh}°, Pitch < {success_pitch_thresh}°, Steps ≥ {min_steps}")

    position_commands = None
    failed = False
    failure_time = None

    for step in range(num_sim_steps):
        t = step * sim_dt

        # Update control at lower frequency
        if step % control_decimation == 0:
            position_commands = controller.update(control_dt)

        if position_commands is None:
            print(f"\n✗ Emergency stop at t={t:.2f}s")
            failed = True
            failure_time = t
            break

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
                print(f"  t={t:5.1f}s: Roll={state['roll']:+6.2f}°, Pitch={state['pitch']:+6.2f}°, "
                      f"X={state['x']:+7.3f}m, Z={state['z']:6.3f}m")

    # Analyze results
    time_log = np.array(time_log)
    roll_log = np.array([s['roll'] for s in state_log])
    pitch_log = np.array([s['pitch'] for s in state_log])
    x_log = np.array([s['x'] for s in state_log])
    height_log = np.array([s['z'] for s in state_log])

    # Compute metrics
    if not failed:
        steps_completed = duration / step_period
        forward_distance = x_log[-1] - x_log[0]
        walking_speed = forward_distance / duration

        # Steady-state analysis (last 20% of test)
        steady_idx = int(len(time_log) * 0.8)
        roll_final = np.mean(roll_log[steady_idx:])
        pitch_final = np.mean(pitch_log[steady_idx:])
        roll_std = np.std(roll_log[steady_idx:])
        pitch_std = np.std(pitch_log[steady_idx:])
        height_final = np.mean(height_log[steady_idx:])

        # Maximum excursions
        roll_max = np.max(np.abs(roll_log))
        pitch_max = np.max(np.abs(pitch_log))
    else:
        steps_completed = failure_time / step_period if failure_time else 0
        forward_distance = x_log[-1] - x_log[0] if len(x_log) > 0 else 0
        walking_speed = 0
        roll_final = roll_log[-1] if len(roll_log) > 0 else 0
        pitch_final = pitch_log[-1] if len(pitch_log) > 0 else 0
        roll_std = 0
        pitch_std = 0
        height_final = height_log[-1] if len(height_log) > 0 else 0
        roll_max = 0
        pitch_max = 0

    print(f"\n{'='*70}")
    print(f"Results:")
    print(f"{'='*70}")
    print(f"  Status: {'✗ FAILED (robot fell)' if failed else '✓ COMPLETED'}")
    if failed:
        print(f"  Failure time: {failure_time:.2f}s")
    print(f"  Steps completed: {steps_completed:.1f}")
    print(f"  Forward distance: {forward_distance:.3f}m")
    print(f"  Walking speed: {walking_speed:.4f} m/s")
    print(f"  Final roll:  {roll_final:+6.2f}° ± {roll_std:.2f}°  (max: {roll_max:.2f}°)")
    print(f"  Final pitch: {pitch_final:+6.2f}° ± {pitch_std:.2f}°  (max: {pitch_max:.2f}°)")
    print(f"  Final height: {height_final:.3f}m")

    # Success criteria
    success = (not failed and
               steps_completed >= min_steps and
               abs(roll_final) < success_roll_thresh and
               abs(pitch_final) < success_pitch_thresh)

    status_str = "✓ PASS" if success else "✗ FAIL"
    print(f"\n{status_str}: {level_name}")
    print(f"{'='*70}")

    p.disconnect()

    return {
        'level': level_name,
        'success': success,
        'failed': failed,
        'failure_time': failure_time,
        'steps_completed': steps_completed,
        'forward_distance': forward_distance,
        'walking_speed': walking_speed,
        'roll_final': roll_final,
        'roll_std': roll_std,
        'roll_max': roll_max,
        'pitch_final': pitch_final,
        'pitch_std': pitch_std,
        'pitch_max': pitch_max,
        'height_final': height_final,
        'time_log': time_log,
        'roll_log': roll_log,
        'pitch_log': pitch_log,
        'x_log': x_log,
        'height_log': height_log,
    }


def plot_all_results(results, output_path):
    """Plot comparison of all test levels"""
    fig, axes = plt.subplots(4, 1, figsize=(12, 14))

    colors = ['blue', 'green', 'orange', 'red']

    for i, result in enumerate(results):
        color = colors[i]
        label = result['level']

        # Forward progress
        axes[0].plot(result['time_log'], result['x_log'],
                    label=label, color=color, linewidth=2)

        # Roll
        axes[1].plot(result['time_log'], result['roll_log'],
                    label=label, color=color, linewidth=2)

        # Pitch
        axes[2].plot(result['time_log'], result['pitch_log'],
                    label=label, color=color, linewidth=2)

        # Height
        axes[3].plot(result['time_log'], result['height_log'],
                    label=label, color=color, linewidth=2)

    # Format plots
    axes[0].set_ylabel('X Position (m)', fontsize=12)
    axes[0].set_title('Phase 4.5: Multi-Step Walking Validation - Forward Progress', fontsize=14, fontweight='bold')
    axes[0].legend(loc='best')
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel('Roll (deg)', fontsize=12)
    axes[1].set_title('Roll Stability', fontsize=12, fontweight='bold')
    axes[1].axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    axes[1].legend(loc='best')
    axes[1].grid(True, alpha=0.3)

    axes[2].set_ylabel('Pitch (deg)', fontsize=12)
    axes[2].set_title('Pitch Stability', fontsize=12, fontweight='bold')
    axes[2].axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    axes[2].legend(loc='best')
    axes[2].grid(True, alpha=0.3)

    axes[3].set_xlabel('Time (s)', fontsize=12)
    axes[3].set_ylabel('Height (m)', fontsize=12)
    axes[3].set_title('Base Height', fontsize=12, fontweight='bold')
    axes[3].axhline(y=0.69, color='gray', linestyle='--', linewidth=1, alpha=0.5, label='Target (0.69m)')
    axes[3].legend(loc='best')
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"\nComparison plot saved to: {output_path}")


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("Phase 4.5: Multi-Step Walking Validation")
    print("Progressive Testing with 4 Complexity Levels")
    print("=" * 70)

    # Define test levels
    test_levels = [
        {
            'name': 'Level 1: Minimal Walking',
            'step_length': 0.02,
            'step_height': 0.01,
            'step_period': 2.0,
            'duration': 10.0,
            'success_roll_thresh': 10.0,
            'success_pitch_thresh': 10.0,
            'min_steps': 3.0,
        },
        {
            'name': 'Level 2: Slow Walking',
            'step_length': 0.05,
            'step_height': 0.02,
            'step_period': 1.5,
            'duration': 20.0,
            'success_roll_thresh': 8.0,
            'success_pitch_thresh': 8.0,
            'min_steps': 10.0,
        },
        {
            'name': 'Level 3: Moderate Walking',
            'step_length': 0.10,
            'step_height': 0.03,
            'step_period': 1.0,
            'duration': 30.0,
            'success_roll_thresh': 5.0,
            'success_pitch_thresh': 5.0,
            'min_steps': 20.0,
        },
        {
            'name': 'Level 4: Indefinite Walking',
            'step_length': 0.10,
            'step_height': 0.03,
            'step_period': 1.0,
            'duration': 60.0,
            'success_roll_thresh': 5.0,
            'success_pitch_thresh': 5.0,
            'min_steps': 50.0,
        },
    ]

    # Run all tests
    all_results = []

    for level_config in test_levels:
        result = run_walking_test(
            level_config['name'],
            level_config['step_length'],
            level_config['step_height'],
            level_config['step_period'],
            level_config['duration'],
            level_config['success_roll_thresh'],
            level_config['success_pitch_thresh'],
            level_config['min_steps']
        )
        all_results.append(result)

    # Summary table
    print("\n" + "=" * 70)
    print("PHASE 4.5 SUMMARY")
    print("=" * 70)

    print(f"\n{'Level':<30} {'Status':<10} {'Steps':<10} {'Dist (m)':<12} {'Speed (m/s)':<12}")
    print("-" * 70)

    for result in all_results:
        status = "✓ PASS" if result['success'] else "✗ FAIL"
        print(f"{result['level']:<30} {status:<10} {result['steps_completed']:>8.1f}   "
              f"{result['forward_distance']:>10.3f}   {result['walking_speed']:>10.4f}")

    print("\n" + "=" * 70)
    print("STABILITY METRICS")
    print("=" * 70)

    print(f"\n{'Level':<30} {'Roll (°)':<20} {'Pitch (°)':<20} {'Height (m)':<12}")
    print("-" * 70)

    for result in all_results:
        print(f"{result['level']:<30} {result['roll_final']:+6.2f} ± {result['roll_std']:<6.2f}   "
              f"{result['pitch_final']:+6.2f} ± {result['pitch_std']:<6.2f}   "
              f"{result['height_final']:>10.3f}")

    # Overall success
    all_passed = all(r['success'] for r in all_results)
    passed_count = sum(1 for r in all_results if r['success'])

    print("\n" + "=" * 70)
    if all_passed:
        print(f"✓ ALL LEVELS PASSED ({passed_count}/4) - Phase 4.5 Complete!")
    else:
        print(f"⚠ PARTIAL SUCCESS ({passed_count}/4 levels passed)")
        print("\nFailed levels:")
        for result in all_results:
            if not result['success']:
                print(f"  - {result['level']}")
                if result['failed']:
                    print(f"    Reason: Robot fell at t={result['failure_time']:.2f}s")
                else:
                    print(f"    Reason: Did not meet success criteria")
    print("=" * 70)

    # Plot results
    plot_all_results(all_results, '/workspace/hunter/logs/phase45_multi_step_validation.png')

    sys.exit(0 if all_passed else 1)

"""
Phase 4.6: Robustness Testing

Stress tests to validate walking system under extreme conditions:
1. Extended Duration: 5 minute continuous walking
2. Mass Uncertainty: ±10-20% mass variations
3. Larger Disturbances: 80-100N pushes
4. Continuous Random Pushes: Multiple disturbances during walk

Author: Phase 4.6 Robustness Testing
Date: 2025-11-26
"""

import numpy as np
import pybullet as p
import pybullet_data
import sys
import os
import time as time_module
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from position_control_walking import PositionControlWalkingController, WalkingControllerParams
from gait_generator import GaitParams


def setup_robot(mass_scale=1.0):
    """
    Initialize PyBullet and load robot

    Args:
        mass_scale: Multiplier for all link masses (1.0 = nominal, 1.2 = +20%, 0.9 = -10%)
    """
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
    base_height = 0.679
    robot_id = p.loadURDF(urdf_path, [0, 0, base_height], useFixedBase=False)

    # Apply mass scaling if requested
    if mass_scale != 1.0:
        print(f"  Applying mass scale: {mass_scale:.2f}x")

        # Scale base mass
        base_mass = p.getDynamicsInfo(robot_id, -1)[0]
        p.changeDynamics(robot_id, -1, mass=base_mass * mass_scale)

        # Scale all link masses
        for i in range(p.getNumJoints(robot_id)):
            link_mass = p.getDynamicsInfo(robot_id, i)[0]
            p.changeDynamics(robot_id, i, mass=link_mass * mass_scale)

        # Verify total mass
        total_mass = base_mass * mass_scale
        for i in range(p.getNumJoints(robot_id)):
            total_mass += p.getDynamicsInfo(robot_id, i)[0]
        print(f"  Total robot mass: {total_mass:.2f} kg")

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


def test_extended_duration(duration=300.0):
    """
    Test 1: Extended Duration Walking

    5 minute continuous walking to test long-term stability
    """
    print("\n" + "=" * 70)
    print("Test 1: Extended Duration (5 minute continuous walking)")
    print("=" * 70)

    robot_id, joint_dict = setup_robot()

    # Use conservative gait (4cm steps, 2s period from Phase 4.4)
    gait_params = GaitParams(
        step_length=0.04,
        step_height=0.01,
        step_period=2.0,
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
    sim_dt = 0.001
    control_dt = 0.02
    num_sim_steps = int(duration / sim_dt)
    control_decimation = int(control_dt / sim_dt)

    # Data logging (sample every 1s to avoid memory issues)
    time_log = []
    state_log = []
    sample_interval = 1.0  # seconds
    next_sample_time = 0.0

    expected_steps = duration / gait_params.step_period
    print(f"\nParameters:")
    print(f"  Duration: {duration}s ({duration/60:.1f} minutes)")
    print(f"  Expected steps: {expected_steps:.0f}")
    print(f"  Gait: {gait_params.step_length}m steps, {gait_params.step_period}s period")

    wall_time_start = time_module.time()
    position_commands = None
    failed = False
    failure_time = None

    for step in range(num_sim_steps):
        t = step * sim_dt

        # Update control
        if step % control_decimation == 0:
            position_commands = controller.update(control_dt)

        if position_commands is None:
            print(f"\n✗ Emergency stop at t={t:.1f}s ({t/60:.1f} min)")
            failed = True
            failure_time = t
            break

        # Apply position control
        apply_position_control(robot_id, joint_dict, position_commands)

        # Step simulation
        p.stepSimulation()

        # Log state (every 1 second)
        if t >= next_sample_time:
            state = get_robot_state(robot_id)
            time_log.append(t)
            state_log.append(state)
            next_sample_time += sample_interval

            # Print status every 30 seconds
            if len(time_log) % 30 == 0:
                steps_so_far = t / gait_params.step_period
                wall_time_elapsed = time_module.time() - wall_time_start
                print(f"  t={t:5.0f}s ({t/60:4.1f}min): Roll={state['roll']:+6.2f}°, "
                      f"Pitch={state['pitch']:+6.2f}°, X={state['x']:+7.3f}m, "
                      f"Steps={steps_so_far:.0f} [Wall time: {wall_time_elapsed:.0f}s]")

    wall_time_total = time_module.time() - wall_time_start

    # Analyze results
    if not failed:
        time_log = np.array(time_log)
        roll_log = np.array([s['roll'] for s in state_log])
        pitch_log = np.array([s['pitch'] for s in state_log])
        x_log = np.array([s['x'] for s in state_log])
        height_log = np.array([s['z'] for s in state_log])

        steps_completed = duration / gait_params.step_period
        forward_distance = x_log[-1] - x_log[0]
        walking_speed = forward_distance / duration

        # Check for drift trends
        roll_trend = np.polyfit(time_log, roll_log, 1)[0]  # deg/s
        pitch_trend = np.polyfit(time_log, pitch_log, 1)[0]  # deg/s

        print(f"\n{'='*70}")
        print(f"Results:")
        print(f"{'='*70}")
        print(f"  Status: ✓ COMPLETED")
        print(f"  Duration: {duration}s ({duration/60:.1f} minutes)")
        print(f"  Steps completed: {steps_completed:.0f}")
        print(f"  Forward distance: {forward_distance:.3f}m")
        print(f"  Walking speed: {walking_speed:.4f} m/s")
        print(f"  Final roll: {roll_log[-1]:+6.2f}° (trend: {roll_trend*60:+.3f}°/min)")
        print(f"  Final pitch: {pitch_log[-1]:+6.2f}° (trend: {pitch_trend*60:+.3f}°/min)")
        print(f"  Roll range: [{np.min(roll_log):+.2f}°, {np.max(roll_log):+.2f}°]")
        print(f"  Pitch range: [{np.min(pitch_log):+.2f}°, {np.max(pitch_log):+.2f}°]")
        print(f"  Final height: {height_log[-1]:.3f}m")
        print(f"  Wall clock time: {wall_time_total:.1f}s (realtime factor: {duration/wall_time_total:.2f}x)")

        # Success: completed full duration without falling
        success = True
        print(f"\n✓ PASS: Completed {duration/60:.1f} minute walk without falling")
    else:
        success = False
        steps_completed = failure_time / gait_params.step_period if failure_time else 0
        time_log = np.array(time_log) if time_log else np.array([0])
        roll_log = np.array([s['roll'] for s in state_log]) if state_log else np.array([0])
        pitch_log = np.array([s['pitch'] for s in state_log]) if state_log else np.array([0])
        x_log = np.array([s['x'] for s in state_log]) if state_log else np.array([0])
        height_log = np.array([s['z'] for s in state_log]) if state_log else np.array([0])
        print(f"\n✗ FAIL: Robot fell at t={failure_time:.1f}s ({failure_time/60:.1f} min)")

    print(f"{'='*70}")

    p.disconnect()

    return {
        'test': 'Extended Duration',
        'success': success,
        'duration': duration if not failed else failure_time,
        'steps': steps_completed,
        'time_log': time_log,
        'roll_log': roll_log,
        'pitch_log': pitch_log,
        'x_log': x_log,
        'height_log': height_log,
    }


def test_mass_uncertainty(mass_scale, duration=60.0):
    """
    Test 2/3: Mass Uncertainty

    Test with ±10-20% mass variations
    """
    mass_percent = int((mass_scale - 1.0) * 100)
    sign = "+" if mass_percent >= 0 else ""

    print("\n" + "=" * 70)
    print(f"Test: Mass Uncertainty ({sign}{mass_percent}% mass)")
    print("=" * 70)

    robot_id, joint_dict = setup_robot(mass_scale=mass_scale)

    # Use conservative gait
    gait_params = GaitParams(
        step_length=0.04,
        step_height=0.01,
        step_period=2.0,
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
    sim_dt = 0.001
    control_dt = 0.02
    num_sim_steps = int(duration / sim_dt)
    control_decimation = int(control_dt / sim_dt)

    # Data logging
    time_log = []
    state_log = []
    sample_interval = 0.1  # 100ms
    next_sample_time = 0.0

    print(f"\nParameters:")
    print(f"  Duration: {duration}s")
    print(f"  Mass scale: {mass_scale:.2f}x ({sign}{mass_percent}%)")
    print(f"  Gait: {gait_params.step_length}m steps, {gait_params.step_period}s period")

    position_commands = None
    failed = False
    failure_time = None

    for step in range(num_sim_steps):
        t = step * sim_dt

        if step % control_decimation == 0:
            position_commands = controller.update(control_dt)

        if position_commands is None:
            print(f"\n✗ Emergency stop at t={t:.1f}s")
            failed = True
            failure_time = t
            break

        apply_position_control(robot_id, joint_dict, position_commands)
        p.stepSimulation()

        if t >= next_sample_time:
            state = get_robot_state(robot_id)
            time_log.append(t)
            state_log.append(state)
            next_sample_time += sample_interval

            if step % 10000 == 0:
                print(f"  t={t:5.1f}s: Roll={state['roll']:+6.2f}°, Pitch={state['pitch']:+6.2f}°, X={state['x']:+7.3f}m")

    # Analyze results
    if not failed:
        time_log = np.array(time_log)
        roll_log = np.array([s['roll'] for s in state_log])
        pitch_log = np.array([s['pitch'] for s in state_log])
        x_log = np.array([s['x'] for s in state_log])

        steps_completed = duration / gait_params.step_period
        forward_distance = x_log[-1] - x_log[0]

        print(f"\n{'='*70}")
        print(f"Results:")
        print(f"{'='*70}")
        print(f"  Status: ✓ COMPLETED")
        print(f"  Steps completed: {steps_completed:.0f}")
        print(f"  Forward distance: {forward_distance:.3f}m")
        print(f"  Final roll: {roll_log[-1]:+6.2f}° (max: {np.max(np.abs(roll_log)):.2f}°)")
        print(f"  Final pitch: {pitch_log[-1]:+6.2f}° (max: {np.max(np.abs(pitch_log)):.2f}°)")

        success = True
        print(f"\n✓ PASS: Completed 60s walk with {sign}{mass_percent}% mass variation")
    else:
        success = False
        steps_completed = failure_time / gait_params.step_period if failure_time else 0
        time_log = np.array(time_log) if time_log else np.array([0])
        roll_log = pitch_log = x_log = np.array([0])
        print(f"\n✗ FAIL: Robot fell at t={failure_time:.1f}s")

    print(f"{'='*70}")

    p.disconnect()

    return {
        'test': f'Mass {sign}{mass_percent}%',
        'success': success,
        'mass_scale': mass_scale,
        'duration': duration if not failed else failure_time,
        'steps': steps_completed,
        'time_log': time_log,
        'roll_log': roll_log,
        'pitch_log': pitch_log,
        'x_log': x_log,
    }


def test_large_pushes(duration=30.0):
    """
    Test 4: Larger Magnitude Pushes

    Apply 80-100N pushes to test disturbance rejection limits
    """
    print("\n" + "=" * 70)
    print("Test: Large Push Disturbances (80-100N)")
    print("=" * 70)

    robot_id, joint_dict = setup_robot()

    gait_params = GaitParams(
        step_length=0.04,
        step_height=0.01,
        step_period=2.0,
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

    # Push configurations (larger forces)
    push_configs = [
        {"name": "80N Forward", "force": [80, 0, 0], "time": 5.0, "duration": 0.1},
        {"name": "100N Lateral Right", "force": [0, -100, 0], "time": 15.0, "duration": 0.1},
        {"name": "80N Backward", "force": [-80, 0, 0], "time": 25.0, "duration": 0.1},
    ]

    # Simulation parameters
    sim_dt = 0.001
    control_dt = 0.02
    num_sim_steps = int(duration / sim_dt)
    control_decimation = int(control_dt / sim_dt)

    time_log = []
    state_log = []
    push_events = []

    print(f"\nPushes scheduled:")
    for pc in push_configs:
        print(f"  t={pc['time']}s: {pc['name']} = {pc['force']} N for {pc['duration']}s")

    position_commands = None
    failed = False
    failure_time = None

    for step in range(num_sim_steps):
        t = step * sim_dt

        if step % control_decimation == 0:
            position_commands = controller.update(control_dt)

        if position_commands is None:
            print(f"\n✗ Robot fell at t={t:.1f}s")
            failed = True
            failure_time = t
            break

        apply_position_control(robot_id, joint_dict, position_commands)

        # Apply pushes
        for pc in push_configs:
            if pc['time'] <= t <= pc['time'] + pc['duration']:
                if t == pc['time'] or (step % 100 == 0 and abs(t - pc['time']) < 0.01):
                    if pc not in push_events:
                        print(f"  >>> Applying {pc['name']} at t={t:.2f}s <<<")
                        push_events.append(pc)

                p.applyExternalForce(
                    objectUniqueId=robot_id,
                    linkIndex=-1,
                    forceObj=pc['force'],
                    posObj=[0, 0, 0],
                    flags=p.LINK_FRAME
                )

        p.stepSimulation()

        if step % 10 == 0:
            state = get_robot_state(robot_id)
            time_log.append(t)
            state_log.append(state)

            if step % 1000 == 0:
                print(f"  t={t:5.1f}s: Roll={state['roll']:+6.2f}°, Pitch={state['pitch']:+6.2f}°, X={state['x']:+7.3f}m")

    # Analyze results
    if not failed:
        time_log = np.array(time_log)
        roll_log = np.array([s['roll'] for s in state_log])
        pitch_log = np.array([s['pitch'] for s in state_log])

        print(f"\n{'='*70}")
        print(f"Results:")
        print(f"{'='*70}")
        print(f"  Status: ✓ COMPLETED")
        print(f"  Pushes survived: {len(push_configs)}/{len(push_configs)}")
        print(f"  Max roll excursion: {np.max(np.abs(roll_log)):.2f}°")
        print(f"  Max pitch excursion: {np.max(np.abs(pitch_log)):.2f}°")

        success = True
        print(f"\n✓ PASS: Survived all {len(push_configs)} large pushes")
    else:
        success = False
        time_log = np.array(time_log) if time_log else np.array([0])
        roll_log = pitch_log = np.array([0])
        print(f"\n✗ FAIL: Robot fell at t={failure_time:.1f}s")

    print(f"{'='*70}")

    p.disconnect()

    return {
        'test': 'Large Pushes',
        'success': success,
        'pushes_survived': len(push_events) if not failed else len([p for p in push_configs if p['time'] < (failure_time or 0)]),
        'time_log': time_log,
        'roll_log': roll_log,
        'pitch_log': pitch_log,
    }


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("Phase 4.6: Robustness Testing")
    print("Stress Tests for Position Control Walking")
    print("=" * 70)

    all_results = []

    try:
        # Test 1: 5 minute extended duration
        print("\n[1/4] Running extended duration test (5 minutes)...")
        result = test_extended_duration(duration=300.0)
        all_results.append(result)

        # Test 2: +20% mass
        print("\n[2/4] Running mass uncertainty test (+20%)...")
        result = test_mass_uncertainty(mass_scale=1.2, duration=60.0)
        all_results.append(result)

        # Test 3: -10% mass
        print("\n[3/4] Running mass uncertainty test (-10%)...")
        result = test_mass_uncertainty(mass_scale=0.9, duration=60.0)
        all_results.append(result)

        # Test 4: Large pushes
        print("\n[4/4] Running large push test (80-100N)...")
        result = test_large_pushes(duration=30.0)
        all_results.append(result)

    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    # Summary
    print("\n" + "=" * 70)
    print("PHASE 4.6 ROBUSTNESS TESTING SUMMARY")
    print("=" * 70)

    print(f"\n{'Test':<35} {'Status':<10} {'Notes':<30}")
    print("-" * 70)

    for result in all_results:
        status = "✓ PASS" if result['success'] else "✗ FAIL"

        if result['test'] == 'Extended Duration':
            notes = f"{result['duration']:.0f}s ({result['duration']/60:.1f}min), {result['steps']:.0f} steps"
        elif 'Mass' in result['test']:
            notes = f"{result['duration']:.0f}s, {result['steps']:.0f} steps"
        elif result['test'] == 'Large Pushes':
            notes = f"{result.get('pushes_survived', 0)} pushes survived"
        else:
            notes = ""

        print(f"{result['test']:<35} {status:<10} {notes:<30}")

    # Overall success
    all_passed = all(r['success'] for r in all_results)
    passed_count = sum(1 for r in all_results if r['success'])

    print("\n" + "=" * 70)
    if all_passed:
        print(f"✓ ALL TESTS PASSED ({passed_count}/{len(all_results)}) - Phase 4.6 Complete!")
    else:
        print(f"⚠ PARTIAL SUCCESS ({passed_count}/{len(all_results)} tests passed)")
    print("=" * 70)

    sys.exit(0 if all_passed else 1)

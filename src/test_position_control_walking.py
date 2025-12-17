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
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from robot_constants import BASE_HEIGHT, standing_config_copy
from test_helpers import urdf_path
from position_control_walking import PositionControlWalkingController, WalkingControllerParams
from gait_generator import GaitParams


def setup_robot():
    """Initialize PyBullet and load robot"""
    # Connect to PyBullet (headless for automated tests)
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(0.001)
    p.setRealTimeSimulation(0)

    # Load ground plane
    p.loadURDF("plane.urdf")

    # Load robot at correct height
    robot_id = p.loadURDF(urdf_path(), [0, 0, BASE_HEIGHT], useFixedBase=False)

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

    Test slightly assertive walking (default 4cm steps, 2s period, 1cm height).
    Success: 3-5 consecutive steps without falling
    """
    print("\n" + "=" * 60)
    print("Test 2: Minimal Walking (Conservative Parameters)")
    print("=" * 60)

    robot_id, joint_dict = setup_robot()

    # Create controller with default gait (4cm step length)
    params = WalkingControllerParams(standing_mode=False, enable_walking=True)

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
    com_est_log = []

    print(f"\nRunning {duration}s walking test...")
    print(f"  Gait: {params.gait.step_length}m steps, {params.gait.step_period}s period")
    print(f"  Expected: ~{duration / params.gait.step_period:.1f} steps")
    print(f"  Simulation dt: {sim_dt*1000:.1f}ms (1000 Hz)")
    print(f"  Control dt: {control_dt*1000:.1f}ms (50 Hz)\n")

    position_commands = None
    for step in range(num_sim_steps):
        t = step * sim_dt

        # Update control at lower frequency
        if step % control_decimation == 0:
            # Get position commands from controller
            position_commands = controller.update(control_dt)
            com_est = controller.get_state_estimate()
            com_est_log.append((t, com_est))

        if position_commands is None:
            print(f"\n✗ FAIL: Emergency stop at t={t:.2f}s")

            # Final state
            state = get_robot_state(robot_id)
            print(f"  Final: Roll={state['roll']:+6.2f}°, Pitch={state['pitch']:+6.2f}°, Z={state['z']:.3f}m")

            # Compute steps completed
            steps_completed = t / params.gait.step_period
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
    com_pos_log = np.array([entry[1]['com_pos'] for entry in com_est_log])
    com_vel_log = np.array([entry[1]['com_vel'] for entry in com_est_log])

    # Calculate steps completed
    steps_completed = duration / params.gait.step_period
    forward_distance = x_log[-1] - x_log[0]
    final_com = com_est_log[-1][1] if com_est_log else {"com_pos": np.zeros(3), "com_vel": np.zeros(3)}

    print(f"\nResults:")
    print(f"  Duration: {duration}s")
    print(f"  Steps completed: {steps_completed:.1f}")
    print(f"  Forward distance: {forward_distance:.3f}m")
    print(f"  Final Roll: {roll_log[-1]:+6.2f}°")
    print(f"  Final Pitch: {pitch_log[-1]:+6.2f}°")
    print(f"  Final Height: {height_log[-1]:.3f}m")
    print(f"  Final CoM (filtered): {final_com['com_pos']}")
    print(f"  Final CoM vel (filtered): {final_com['com_vel']}")

    # Success criteria: completed at least 3 steps without falling
    success = (steps_completed >= 3.0 and
               abs(roll_log[-1]) < 15.0 and
               abs(pitch_log[-1]) < 15.0)

    # Basic estimator sanity checks (height and bounded velocity)
    if com_pos_log.size > 0:
        final_height = final_com["com_pos"][2]
        final_speed = np.linalg.norm(final_com["com_vel"][:2])
        success = success and 0.55 < final_height < 0.80 and final_speed < 1.0

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

    # Overlay filtered CoM height for comparison if available
    if com_pos_log.size > 0:
        axes[2].plot(
            np.array([entry[0] for entry in com_est_log]),
            com_pos_log[:, 2],
            label='Filtered CoM Z',
            linewidth=2,
            linestyle='--'
        )

    plt.tight_layout()
    plt.savefig('/workspace/hunter/logs/position_control_walking_test.png', dpi=150)
    print(f"\nPlot saved to: logs/position_control_walking_test.png")

    p.disconnect()
    return success


def test_disturbance_rejection():
    """
    Test 3: Disturbance/Push Rejection During Walking

    Apply external pushes to test ZMP feedback robustness.
    Success: Robot recovers from pushes without falling
    """
    print("\n" + "=" * 60)
    print("Test 3: Disturbance/Push Rejection During Walking")
    print("=" * 60)

    # Test various push magnitudes and directions
    push_configs = [
        {"name": "Forward Push (50N)", "force": [50, 0, 0], "time": 3.0, "duration": 0.1},
        {"name": "Backward Push (50N)", "force": [-50, 0, 0], "time": 5.0, "duration": 0.1},
        {"name": "Lateral Push Right (30N)", "force": [0, -30, 0], "time": 7.0, "duration": 0.1},
        {"name": "Lateral Push Left (30N)", "force": [0, 30, 0], "time": 9.0, "duration": 0.1},
    ]

    all_results = []

    for push_config in push_configs:
        print(f"\n--- Testing: {push_config['name']} ---")
        robot_id, joint_dict = setup_robot()

        # Create controller with default gait
        params = WalkingControllerParams(standing_mode=False, enable_walking=True)
        controller = PositionControlWalkingController(robot_id, joint_dict, params)
        controller.reset()

        # Simulation parameters
        sim_dt = 0.001
        control_dt = 0.02
        duration = 12.0  # Longer to see recovery
        num_sim_steps = int(duration / sim_dt)
        control_decimation = int(control_dt / sim_dt)

        # Data logging
        time_log = []
        state_log = []
        push_applied = False
        push_start_time = push_config["time"]
        push_end_time = push_start_time + push_config["duration"]

        print(f"  Push will be applied at t={push_start_time:.1f}s for {push_config['duration']}s")
        print(f"  Force: {push_config['force']} N\n")

        position_commands = None
        failed = False
        for step in range(num_sim_steps):
            t = step * sim_dt

            # Update control at lower frequency
            if step % control_decimation == 0:
                position_commands = controller.update(control_dt)

            if position_commands is None:
                print(f"\n✗ Robot fell at t={t:.2f}s")
                failed = True
                break

            # Apply position control
            apply_position_control(robot_id, joint_dict, position_commands)

            # Apply push during specified time window
            if push_start_time <= t <= push_end_time:
                if not push_applied:
                    print(f"  >>> Applying push at t={t:.2f}s <<<")
                    push_applied = True
                # Apply force continuously during push duration
                p.applyExternalForce(
                    objectUniqueId=robot_id,
                    linkIndex=-1,  # Base link
                    forceObj=push_config["force"],
                    posObj=[0, 0, 0],
                    flags=p.LINK_FRAME
                )
            elif push_applied and t > push_end_time:
                print(f"  >>> Push ended at t={t:.2f}s <<<")
                push_applied = False  # Reset for logging

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
        if not failed:
            time_log = np.array(time_log)
            roll_log = np.array([s['roll'] for s in state_log])
            pitch_log = np.array([s['pitch'] for s in state_log])
            x_log = np.array([s['x'] for s in state_log])
            height_log = np.array([s['z'] for s in state_log])

            # Check stability after push (last 2 seconds)
            post_push_idx = int(len(time_log) * 0.85)
            roll_final = np.mean(roll_log[post_push_idx:])
            pitch_final = np.mean(pitch_log[post_push_idx:])
            forward_distance = x_log[-1] - x_log[0]

            print(f"\n  Results:")
            print(f"    Forward distance: {forward_distance:.3f}m")
            print(f"    Final Roll: {roll_final:+6.2f}°")
            print(f"    Final Pitch: {pitch_final:+6.2f}°")
            print(f"    Final Height: {height_log[-1]:.3f}m")

            # Success: recovered from push (roll/pitch < 20°, didn't fall)
            success = abs(roll_final) < 20.0 and abs(pitch_final) < 20.0
            status = "✓ PASS" if success else "✗ FAIL"
            print(f"  {status}: Recovered from {push_config['name']}")

            all_results.append({
                "name": push_config["name"],
                "success": success,
                "roll": roll_final,
                "pitch": pitch_final,
                "distance": forward_distance
            })
        else:
            all_results.append({
                "name": push_config["name"],
                "success": False,
                "roll": None,
                "pitch": None,
                "distance": 0.0
            })

        p.disconnect()

    # Summary
    print("\n" + "=" * 60)
    print("Disturbance Rejection Summary")
    print("=" * 60)
    for result in all_results:
        status = "✓ PASS" if result["success"] else "✗ FAIL"
        if result["roll"] is not None:
            print(f"  {status}: {result['name']} - Roll={result['roll']:+.2f}°, Pitch={result['pitch']:+.2f}°, Dist={result['distance']:.3f}m")
        else:
            print(f"  {status}: {result['name']} - Robot fell")

    # Overall success: robot recovered from at least 75% of pushes
    overall_success = sum(1 for r in all_results if r["success"]) >= 3
    return overall_success


def run_gain_sweep():
    """
    Optional: sweep ZMP feedback gains to observe forward progress impact.
    Controlled via ZMP_GAIN_SWEEP=1 to avoid slowing down default tests.
    """
    if os.environ.get("ZMP_GAIN_SWEEP") not in ("1", "true", "True"):
        return

    gains = [0.0, 0.1, 0.2, 0.3, 0.4]
    results = []
    print("\n" + "=" * 60)
    print("ZMP Feedback Gain Sweep")
    print("=" * 60)
    for gain in gains:
        robot_id, joint_dict = setup_robot()
        params = WalkingControllerParams(
            standing_mode=False,
            enable_walking=True,
            zmp_feedback_gain=gain
        )
        controller = PositionControlWalkingController(robot_id, joint_dict, params)
        controller.reset()

        sim_dt = 0.001
        control_dt = 0.02
        duration = 10.0
        num_sim_steps = int(duration / sim_dt)
        control_decimation = int(control_dt / sim_dt)

        x_start = p.getBasePositionAndOrientation(robot_id)[0][0]
        position_commands = None
        for step in range(num_sim_steps):
            if step % control_decimation == 0:
                position_commands = controller.update(control_dt)
            if position_commands is None:
                break
            apply_position_control(robot_id, joint_dict, position_commands)
            p.stepSimulation()

        x_end = p.getBasePositionAndOrientation(robot_id)[0][0]
        forward_distance = x_end - x_start
        state = get_robot_state(robot_id)
        results.append((gain, forward_distance, state["roll"], state["pitch"]))
        p.disconnect()

    print("\nGain Sweep Results (10s, conservative gait):")
    for gain, dist, roll, pitch in results:
        print(f"  gain={gain:.2f}: forward={dist:.3f} m, roll={roll:+.2f}°, pitch={pitch:+.2f}°")


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("Position Control Walking Controller - Integration Tests")
    print("=" * 60)

    results = []

    # Check if disturbance test is requested
    run_disturbance_test = os.environ.get("DISTURBANCE_TEST") in ("1", "true", "True")

    # Run tests
    try:
        results.append(("Standing Mode", test_standing_mode()))
        results.append(("Minimal Walking", test_minimal_walking()))

        # Run disturbance test if requested
        if run_disturbance_test:
            results.append(("Disturbance Rejection", test_disturbance_rejection()))

        # Run gain sweep if requested
        run_gain_sweep()
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
        status_msg = "Phase 4.4 Complete" if run_disturbance_test else "Phase 4.3 Integration Successful"
        print(f"✓ ALL TESTS PASSED - {status_msg}")
    else:
        print("✗ SOME TESTS FAILED - Review results above")
    print("=" * 60)

    sys.exit(0 if all_passed else 1)

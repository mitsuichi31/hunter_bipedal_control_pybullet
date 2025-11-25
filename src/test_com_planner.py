"""
Test script for CoM Trajectory Planner

Validates Preview Control implementation without robot simulation.
Tests ZMP tracking, trajectory smoothness, and stability.

Author: Phase 4.1 Testing
Date: 2025-11-25
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for Docker
import matplotlib.pyplot as plt
from com_planner_simple import SimpleCoMPlanner2D, SimpleCoMPlannerParams


def test_step_response():
    """
    Test 1: Step Response

    Test planner's response to step change in desired ZMP.
    Should produce smooth CoM trajectory that tracks ZMP.
    """
    print("=" * 60)
    print("Test 1: Step Response")
    print("=" * 60)

    params = SimpleCoMPlannerParams(
        com_height=0.689,
        zmp_kp=10.0,
        zmp_kd=3.0,
        preview_time=0.5,
        dt=0.01
    )

    planner = SimpleCoMPlanner2D(params)

    # Test scenario: Step from 0 to 0.1m ZMP
    duration = 3.0  # seconds
    N = int(duration / params.dt)

    zmp_ref = np.zeros((N, 2))
    zmp_ref[100:, 0] = 0.1  # Step at t=1.0s in X direction
    zmp_ref[100:, 1] = 0.05  # Step at t=1.0s in Y direction

    # Plan trajectory
    initial_state = (np.array([0.0, 0.0]), np.array([0.0, 0.0]))
    trajectory = planner.plan_trajectory(zmp_ref, initial_state)

    # Extract results
    com_x = trajectory[:, 0, 0]
    com_y = trajectory[:, 1, 0]
    vel_x = trajectory[:, 0, 1]
    vel_y = trajectory[:, 1, 1]
    acc_x = trajectory[:, 0, 2]
    acc_y = trajectory[:, 1, 2]

    # Compute actual ZMP from CoM
    omega2 = params.omega ** 2
    zmp_x = com_x - acc_x / omega2
    zmp_y = com_y - acc_y / omega2

    # Analyze results
    time = np.arange(N) * params.dt

    # Tracking error after settling (last 1 second)
    settle_idx = int(2.0 / params.dt)
    tracking_error_x = np.mean(np.abs(zmp_ref[settle_idx:, 0] - zmp_x[settle_idx:]))
    tracking_error_y = np.mean(np.abs(zmp_ref[settle_idx:, 1] - zmp_y[settle_idx:]))

    print(f"\nResults:")
    print(f"  Final CoM X: {com_x[-1]:.6f} m (ZMP ref: {zmp_ref[-1, 0]:.6f} m)")
    print(f"  Final CoM Y: {com_y[-1]:.6f} m (ZMP ref: {zmp_ref[-1, 1]:.6f} m)")
    print(f"  Final ZMP X: {zmp_x[-1]:.6f} m (error: {abs(zmp_ref[-1, 0] - zmp_x[-1]):.6f} m)")
    print(f"  Final ZMP Y: {zmp_y[-1]:.6f} m (error: {abs(zmp_ref[-1, 1] - zmp_y[-1]):.6f} m)")
    print(f"  Tracking error X (settled): {tracking_error_x:.6f} m")
    print(f"  Tracking error Y (settled): {tracking_error_y:.6f} m")
    print(f"  Max velocity X: {np.max(np.abs(vel_x)):.3f} m/s")
    print(f"  Max acceleration X: {np.max(np.abs(acc_x)):.3f} m/s^2")

    # Check success criteria
    success = tracking_error_x < 0.01 and tracking_error_y < 0.01
    print(f"\n{'✓ PASS' if success else '✗ FAIL'}: Tracking error < 1cm")

    # Plot results
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))

    # X direction
    axes[0, 0].plot(time, com_x, label='CoM X', linewidth=2)
    axes[0, 0].plot(time, zmp_ref[:, 0], 'r--', label='ZMP ref X', linewidth=1)
    axes[0, 0].plot(time, zmp_x, 'g:', label='ZMP actual X', linewidth=2)
    axes[0, 0].set_ylabel('Position (m)')
    axes[0, 0].set_title('X Direction - Position')
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    axes[1, 0].plot(time, vel_x, linewidth=2)
    axes[1, 0].set_ylabel('Velocity (m/s)')
    axes[1, 0].set_title('X Direction - Velocity')
    axes[1, 0].grid(True)

    axes[2, 0].plot(time, acc_x, linewidth=2)
    axes[2, 0].set_ylabel('Acceleration (m/s²)')
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_title('X Direction - Acceleration')
    axes[2, 0].grid(True)

    # Y direction
    axes[0, 1].plot(time, com_y, label='CoM Y', linewidth=2)
    axes[0, 1].plot(time, zmp_ref[:, 1], 'r--', label='ZMP ref Y', linewidth=1)
    axes[0, 1].plot(time, zmp_y, 'g:', label='ZMP actual Y', linewidth=2)
    axes[0, 1].set_ylabel('Position (m)')
    axes[0, 1].set_title('Y Direction - Position')
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    axes[1, 1].plot(time, vel_y, linewidth=2)
    axes[1, 1].set_ylabel('Velocity (m/s)')
    axes[1, 1].set_title('Y Direction - Velocity')
    axes[1, 1].grid(True)

    axes[2, 1].plot(time, acc_y, linewidth=2)
    axes[2, 1].set_ylabel('Acceleration (m/s²)')
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].set_title('Y Direction - Acceleration')
    axes[2, 1].grid(True)

    plt.tight_layout()
    plt.savefig('/workspace/hunter/logs/com_planner_step_response.png', dpi=150)
    print(f"\nPlot saved to: logs/com_planner_step_response.png")

    return success


def test_walking_gait():
    """
    Test 2: Walking Gait ZMP Trajectory

    Simulate ZMP trajectory for walking gait:
    - Double support: ZMP transitions between feet
    - Single support: ZMP at stance foot
    """
    print("\n" + "=" * 60)
    print("Test 2: Walking Gait ZMP Tracking")
    print("=" * 60)

    params = SimpleCoMPlannerParams(
        com_height=0.689,
        zmp_kp=10.0,
        zmp_kd=3.0,
        preview_time=0.5,
        dt=0.01
    )

    planner = SimpleCoMPlanner2D(params)

    # Simulate walking parameters
    step_period = 1.5  # seconds per step
    step_length = 0.10  # meters
    foot_spacing = 0.18  # meters (lateral spacing between feet)
    double_support_ratio = 0.4

    duration = 6.0  # 4 steps
    N = int(duration / params.dt)
    time = np.arange(N) * params.dt

    # Generate ZMP reference for walking
    zmp_ref = np.zeros((N, 2))

    # Initial standing
    zmp_ref[:, 0] = 0.0  # X: centered
    zmp_ref[:, 1] = 0.0  # Y: centered

    # Simulate 4 steps
    current_x = 0.0
    for step in range(4):
        t_start = step * step_period
        t_end = (step + 1) * step_period
        idx_start = int(t_start / params.dt)
        idx_end = int(t_end / params.dt)

        double_support_duration = step_period * double_support_ratio
        idx_ds_end = int((t_start + double_support_duration) / params.dt)

        # Foot positions
        if step % 2 == 0:  # Left foot swing
            stance_y = -foot_spacing / 2  # Right foot
            swing_y = foot_spacing / 2  # Left foot
        else:  # Right foot swing
            stance_y = foot_spacing / 2  # Left foot
            swing_y = -foot_spacing / 2  # Right foot

        target_x = current_x + step_length

        # Double support: ZMP transitions smoothly
        for i in range(idx_start, min(idx_ds_end, idx_end)):
            ratio = (i - idx_start) / (idx_ds_end - idx_start + 1)
            zmp_ref[i, 0] = current_x * (1 - ratio) + target_x * ratio
            zmp_ref[i, 1] = zmp_ref[idx_start - 1, 1] * (1 - ratio) + stance_y * ratio

        # Single support: ZMP at stance foot
        for i in range(idx_ds_end, idx_end):
            zmp_ref[i, 0] = target_x
            zmp_ref[i, 1] = stance_y

        current_x = target_x

    # Plan CoM trajectory
    initial_state = (np.array([0.0, 0.0]), np.array([0.0, 0.0]))
    trajectory = planner.plan_trajectory(zmp_ref, initial_state)

    # Extract results
    com_x = trajectory[:, 0, 0]
    com_y = trajectory[:, 1, 0]
    acc_x = trajectory[:, 0, 2]
    acc_y = trajectory[:, 1, 2]

    # Compute actual ZMP
    omega2 = params.omega ** 2
    zmp_x = com_x - acc_x / omega2
    zmp_y = com_y - acc_y / omega2

    # Analyze tracking
    tracking_error_x = np.mean(np.abs(zmp_ref[:, 0] - zmp_x))
    tracking_error_y = np.mean(np.abs(zmp_ref[:, 1] - zmp_y))
    max_error_x = np.max(np.abs(zmp_ref[:, 0] - zmp_x))
    max_error_y = np.max(np.abs(zmp_ref[:, 1] - zmp_y))

    print(f"\nResults:")
    print(f"  Final CoM X: {com_x[-1]:.3f} m (traveled {com_x[-1]:.3f} m in 4 steps)")
    print(f"  Mean tracking error X: {tracking_error_x:.6f} m")
    print(f"  Mean tracking error Y: {tracking_error_y:.6f} m")
    print(f"  Max tracking error X: {max_error_x:.6f} m")
    print(f"  Max tracking error Y: {max_error_y:.6f} m")

    # Check success
    success = max_error_x < 0.02 and max_error_y < 0.02
    print(f"\n{'✓ PASS' if success else '✗ FAIL'}: Max tracking error < 2cm")

    # Plot
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    # ZMP trajectory (top view)
    axes[0, 0].plot(zmp_ref[:, 0], zmp_ref[:, 1], 'r--', label='ZMP reference', linewidth=2)
    axes[0, 0].plot(zmp_x, zmp_y, 'g-', label='ZMP actual', linewidth=1.5, alpha=0.7)
    axes[0, 0].plot(com_x, com_y, 'b-', label='CoM', linewidth=1.5)
    axes[0, 0].set_xlabel('X (m)')
    axes[0, 0].set_ylabel('Y (m)')
    axes[0, 0].set_title('Top View: CoM and ZMP Trajectories')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    axes[0, 0].axis('equal')

    # X over time
    axes[0, 1].plot(time, com_x, 'b-', label='CoM X', linewidth=2)
    axes[0, 1].plot(time, zmp_ref[:, 0], 'r--', label='ZMP ref X', linewidth=1)
    axes[0, 1].plot(time, zmp_x, 'g:', label='ZMP actual X', linewidth=2)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('X Position (m)')
    axes[0, 1].set_title('Forward Progress')
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    # Y over time
    axes[1, 0].plot(time, com_y, 'b-', label='CoM Y', linewidth=2)
    axes[1, 0].plot(time, zmp_ref[:, 1], 'r--', label='ZMP ref Y', linewidth=1)
    axes[1, 0].plot(time, zmp_y, 'g:', label='ZMP actual Y', linewidth=2)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Y Position (m)')
    axes[1, 0].set_title('Lateral Motion')
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    # Tracking error over time
    axes[1, 1].plot(time, np.abs(zmp_ref[:, 0] - zmp_x) * 1000, label='Error X (mm)', linewidth=1.5)
    axes[1, 1].plot(time, np.abs(zmp_ref[:, 1] - zmp_y) * 1000, label='Error Y (mm)', linewidth=1.5)
    axes[1, 1].axhline(y=20, color='r', linestyle='--', label='20mm threshold')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('ZMP Tracking Error (mm)')
    axes[1, 1].set_title('Tracking Error')
    axes[1, 1].legend()
    axes[1, 1].grid(True)

    plt.tight_layout()
    plt.savefig('/workspace/hunter/logs/com_planner_walking_gait.png', dpi=150)
    print(f"\nPlot saved to: logs/com_planner_walking_gait.png")

    return success


def test_computational_performance():
    """
    Test 3: Computational Performance

    Measure computation time to ensure real-time capability.
    """
    print("\n" + "=" * 60)
    print("Test 3: Computational Performance")
    print("=" * 60)

    import time

    params = SimpleCoMPlannerParams(
        com_height=0.689,
        zmp_kp=10.0,
        zmp_kd=3.0,
        preview_time=0.5,
        dt=0.01
    )

    planner = SimpleCoMPlanner2D(params)

    # Generate random ZMP trajectory
    N = 1000
    zmp_ref = np.random.randn(N, 2) * 0.05

    # Measure planning time
    start = time.time()
    trajectory = planner.plan_trajectory(zmp_ref)
    elapsed = time.time() - start

    # Measure single-step time
    planner.reset(np.array([0.0, 0.0]), np.array([0.0, 0.0]))
    single_step_times = []
    for i in range(100):
        start = time.time()
        planner.compute_com_command(np.array([0.0, 0.0]))
        single_step_times.append(time.time() - start)

    avg_step_time = np.mean(single_step_times) * 1000  # ms
    max_step_time = np.max(single_step_times) * 1000  # ms

    print(f"\nResults:")
    print(f"  Full trajectory (N={N}): {elapsed:.3f} s ({N/elapsed:.1f} steps/s)")
    print(f"  Single step (avg): {avg_step_time:.3f} ms")
    print(f"  Single step (max): {max_step_time:.3f} ms")
    print(f"  Required for 100 Hz: < 10 ms per step")

    success = avg_step_time < 10.0
    print(f"\n{'✓ PASS' if success else '✗ FAIL'}: Fast enough for 100 Hz control")

    return success


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("CoM Planner Validation Tests")
    print("=" * 60)

    results = []

    # Run tests
    results.append(("Step Response", test_step_response()))
    results.append(("Walking Gait", test_walking_gait()))
    results.append(("Performance", test_computational_performance()))

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
        print("✓ ALL TESTS PASSED - CoM Planner Ready for Integration")
    else:
        print("✗ SOME TESTS FAILED - Review results above")
    print("=" * 60)

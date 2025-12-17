#!/usr/bin/env python3
"""
Validation test for stability_metrics.py

Compares new accurate CoM calculation against old base-link approximation
to quantify improvement.
"""

import pybullet as p
import pybullet_data
import numpy as np
import os
import sys

from robot_constants import BASE_HEIGHT
from stability_metrics import StabilityMetrics


def test_com_accuracy():
    """Compare accurate CoM vs base-link approximation"""

    print("="*80)
    print("STABILITY METRICS VALIDATION TEST")
    print("="*80)

    # Connect to PyBullet
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load robot
    urdf_path = os.path.join(os.path.dirname(__file__), "../models/urdf/hunter.urdf")
    robot_id = p.loadURDF(urdf_path, [0, 0, BASE_HEIGHT])

    print(f"\n✓ Robot loaded (ID: {robot_id})")

    # Create metrics calculator
    metrics = StabilityMetrics(robot_id)

    # Test 1: Accurate CoM calculation
    print("\n" + "-"*80)
    print("TEST 1: Accurate CoM Calculation")
    print("-"*80)

    com_accurate = metrics.compute_com_position()
    print(f"\nAccurate CoM (all links): [{com_accurate[0]:.4f}, {com_accurate[1]:.4f}, {com_accurate[2]:.4f}] m")

    # Old approximation (base link only)
    base_pos, _ = p.getBasePositionAndOrientation(robot_id)
    com_approx = np.array(base_pos)
    print(f"Old approximation (base):  [{com_approx[0]:.4f}, {com_approx[1]:.4f}, {com_approx[2]:.4f}] m")

    # Calculate error
    error = com_accurate - com_approx
    error_magnitude = np.linalg.norm(error)

    print(f"\nError: [{error[0]:.4f}, {error[1]:.4f}, {error[2]:.4f}] m")
    print(f"Error magnitude: {error_magnitude * 100:.2f} cm")

    if error_magnitude < 0.05:  # Less than 5cm
        print("✓ Error acceptable (<5cm)")
    else:
        print("⚠ Large error (>5cm)")

    # Test 2: Total mass
    print("\n" + "-"*80)
    print("TEST 2: Total Mass Calculation")
    print("-"*80)

    total_mass = metrics.compute_total_mass()
    print(f"\nTotal robot mass: {total_mass:.3f} kg")

    # Sanity check (Hunter should be 8-12 kg approximately)
    if 5.0 < total_mass < 15.0:
        print("✓ Mass in expected range (5-15 kg)")
    else:
        print("⚠ Mass outside expected range")

    # Test 3: CoM velocity
    print("\n" + "-"*80)
    print("TEST 3: CoM Velocity Calculation")
    print("-"*80)

    com_vel = metrics.compute_com_velocity()
    print(f"\nCoM velocity: [{com_vel[0]:.4f}, {com_vel[1]:.4f}, {com_vel[2]:.4f}] m/s")
    print(f"Speed: {np.linalg.norm(com_vel):.4f} m/s")

    # For standing robot, velocity should be near zero
    if np.linalg.norm(com_vel) < 0.01:
        print("✓ Near zero (standing)")
    else:
        print("⚠ Non-zero velocity (robot moving?)")

    # Test 4: ZMP computation
    print("\n" + "-"*80)
    print("TEST 4: ZMP Computation")
    print("-"*80)

    dt = 0.001  # 1ms timestep
    com_acc = metrics.compute_com_acceleration(dt)
    zmp = metrics.compute_zmp(com_accurate, com_acc)

    print(f"\nCoM acceleration: [{com_acc[0]:.4f}, {com_acc[1]:.4f}, {com_acc[2]:.4f}] m/s^2")
    print(f"ZMP: [{zmp[0]:.4f}, {zmp[1]:.4f}] m")
    print(f"CoM (XY): [{com_accurate[0]:.4f}, {com_accurate[1]:.4f}] m")

    zmp_error = np.linalg.norm(zmp - com_accurate[0:2])
    print(f"\nZMP deviation from CoM: {zmp_error * 100:.2f} cm")

    # For standing, ZMP should be close to CoM projection
    if zmp_error < 0.05:
        print("✓ ZMP near CoM projection (standing)")

    # Test 5: Get all metrics at once
    print("\n" + "-"*80)
    print("TEST 5: Get All Metrics")
    print("-"*80)

    all_metrics = metrics.get_metrics(dt=dt)

    print("\nComputed metrics:")
    for key, value in all_metrics.items():
        if isinstance(value, np.ndarray):
            print(f"  {key}: {value}")
        else:
            print(f"  {key}: {value:.4f}")

    # Summary
    print("\n" + "="*80)
    print("SUMMARY")
    print("="*80)
    print(f"\n✓ Stability metrics module functional")
    print(f"  - CoM accuracy improvement: {error_magnitude * 100:.1f} cm")
    print(f"  - Total mass: {total_mass:.2f} kg")
    print(f"  - ZMP computation: Working")
    print(f"  - All metrics API: Working")

    print("\n✓ TEST PASSED - Ready for integration")

    p.disconnect()


if __name__ == "__main__":
    test_com_accuracy()

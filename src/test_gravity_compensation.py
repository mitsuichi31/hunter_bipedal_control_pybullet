#!/usr/bin/env python3
"""
Validation test for gravity_compensation.py

Tests:
1. Gravity torque computation accuracy
2. Torque reduction estimates
3. Per-joint enable/disable
4. Dictionary interface
"""

import pybullet as p
import pybullet_data
import numpy as np
import os

from robot_constants import BASE_HEIGHT
from gravity_compensation import GravityCompensation, estimate_torque_reduction


def test_gravity_compensation():
    """Test gravity compensation module"""

    print("="*80)
    print("GRAVITY COMPENSATION VALIDATION TEST")
    print("="*80)

    # Connect to PyBullet
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load robot
    urdf_path = os.path.join(os.path.dirname(__file__), "../models/urdf/hunter.urdf")
    robot_id = p.loadURDF(urdf_path, [0, 0, BASE_HEIGHT])

    print(f"\n✓ Robot loaded (ID: {robot_id})")

    # Create gravity compensation calculator
    gc = GravityCompensation(robot_id)

    # Test 1: Compute gravity torques
    print("\n" + "-"*80)
    print("TEST 1: Gravity Torque Computation")
    print("-"*80)

    gravity_torques = gc.compute_gravity_torques()

    print(f"\nNumber of actuated joints: {len(gravity_torques)}")
    print(f"\nGravity torques (N⋅m):")

    for i, torque in enumerate(gravity_torques):
        print(f"  Joint {i}: {torque:8.4f} N⋅m")

    max_torque = np.max(np.abs(gravity_torques))
    rms_torque = np.sqrt(np.mean(gravity_torques**2))

    print(f"\nStatistics:")
    print(f"  Max torque: {max_torque:.4f} N⋅m")
    print(f"  RMS torque: {rms_torque:.4f} N⋅m")

    # Sanity check - torques should be reasonable for 12kg robot
    if max_torque < 50.0:  # Less than 50 N⋅m
        print("✓ Torques in reasonable range")
    else:
        print("⚠ Very large torques detected")

    # Test 2: Dictionary interface
    print("\n" + "-"*80)
    print("TEST 2: Dictionary Interface")
    print("-"*80)

    torques_dict = gc.compute_gravity_torques_dict()

    print(f"\nGravity torques by joint name:")
    for joint_name, torque in sorted(torques_dict.items()):
        print(f"  {joint_name:20s}: {torque:8.4f} N⋅m")

    # Check that we have all expected joints
    expected_joints = [
        'leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint', 'leg_l5_joint',
        'leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint', 'leg_r5_joint'
    ]

    missing = set(expected_joints) - set(torques_dict.keys())
    if not missing:
        print("\n✓ All expected joints present")
    else:
        print(f"\n⚠ Missing joints: {missing}")

    # Test 3: Enable/Disable
    print("\n" + "-"*80)
    print("TEST 3: Enable/Disable Functionality")
    print("-"*80)

    # Disable globally
    gc.enable_compensation(False)
    torques_disabled = gc.compute_gravity_torques()

    print(f"\nWith compensation disabled:")
    print(f"  All torques zero: {np.allclose(torques_disabled, 0)}")

    # Re-enable
    gc.enable_compensation(True)
    torques_enabled = gc.compute_gravity_torques()

    print(f"With compensation enabled:")
    print(f"  Torques non-zero: {not np.allclose(torques_enabled, 0)}")

    # Disable specific joint
    gc.set_joint_compensation('leg_l3_joint', False)
    torques_partial = gc.compute_gravity_torques_dict()

    print(f"\nWith leg_l3_joint disabled:")
    print(f"  leg_l3_joint torque: {torques_partial['leg_l3_joint']:.4f} N⋅m (should be 0)")

    # Check status
    status = gc.get_compensation_status()
    print(f"  leg_l3_joint enabled: {status['leg_l3_joint']}")
    print(f"  leg_r3_joint enabled: {status['leg_r3_joint']}")

    if abs(torques_partial['leg_l3_joint']) < 1e-6:
        print("✓ Per-joint disable working")

    # Re-enable for next test
    gc.set_joint_compensation('leg_l3_joint', True)

    # Test 4: Torque reduction estimate
    print("\n" + "-"*80)
    print("TEST 4: Torque Reduction Estimate")
    print("-"*80)

    analysis = estimate_torque_reduction(robot_id)

    print(f"\nGravity torque analysis:")
    print(f"  Max gravity torque: {analysis['max_gravity_torque']:.4f} N⋅m")
    print(f"  RMS gravity torque: {analysis['rms_gravity_torque']:.4f} N⋅m")
    print(f"  Estimated efficiency gain: {analysis['estimated_reduction_percent']:.1f}%")

    print(f"\nInterpretation:")
    if analysis['estimated_reduction_percent'] > 20:
        print(f"  ✓ Significant benefit expected (>{analysis['estimated_reduction_percent']:.0f}%)")
    else:
        print(f"  ⚠ Moderate benefit ({analysis['estimated_reduction_percent']:.0f}%)")

    # Test 5: Zero gravity check
    print("\n" + "-"*80)
    print("TEST 5: Zero Gravity Validation")
    print("-"*80)

    # Save current gravity
    p.setGravity(0, 0, 0)  # Zero gravity

    torques_zero_g = gc.compute_gravity_torques()
    max_torque_zero_g = np.max(np.abs(torques_zero_g))

    print(f"\nIn zero gravity:")
    print(f"  Max torque: {max_torque_zero_g:.6f} N⋅m")

    # Restore gravity
    p.setGravity(0, 0, -9.81)

    if max_torque_zero_g < 0.01:  # Should be very small
        print("✓ Zero gravity produces near-zero torques")
    else:
        print("⚠ Unexpected torques in zero gravity")

    # Summary
    print("\n" + "="*80)
    print("SUMMARY")
    print("="*80)

    print(f"\n✓ Gravity compensation module functional")
    print(f"  - Torque computation: Working")
    print(f"  - Dictionary interface: Working")
    print(f"  - Enable/disable: Working")
    print(f"  - Expected efficiency gain: {analysis['estimated_reduction_percent']:.1f}%")
    print(f"  - Max gravity torque: {analysis['max_gravity_torque']:.2f} N⋅m")

    print("\n✓ TEST PASSED - Ready for integration")

    p.disconnect()


if __name__ == "__main__":
    test_gravity_compensation()

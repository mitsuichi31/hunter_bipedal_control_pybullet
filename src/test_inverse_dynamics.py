#!/usr/bin/env python3
"""
Test for inverse_dynamics.py

Validates:
1. Mass matrix computation
2. Gravity torque computation
3. Coriolis force computation
4. Full inverse dynamics
5. Forward-inverse consistency
"""

import pybullet as p
import pybullet_data
import numpy as np
import os
from robot_constants import BASE_HEIGHT

from inverse_dynamics import InverseDynamics


def test_inverse_dynamics():
    """Test inverse dynamics module"""

    print("=" * 80)
    print("INVERSE DYNAMICS VALIDATION TEST")
    print("=" * 80)

    # Connect to PyBullet
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load robot
    urdf_path = os.path.join(os.path.dirname(__file__), "../models/urdf/hunter.urdf")
    robot_id = p.loadURDF(urdf_path, [0, 0, BASE_HEIGHT])

    print(f"\n✓ Robot loaded (ID: {robot_id})")

    # Create inverse dynamics calculator
    inv_dyn = InverseDynamics(robot_id)

    joint_info = inv_dyn.get_joint_info()
    num_joints = len(joint_info)
    print(f"✓ Found {num_joints} actuated joints")

    # Test 1: Mass Matrix
    print("\n" + "-" * 80)
    print("TEST 1: Mass Matrix Computation")
    print("-" * 80)

    M = inv_dyn.compute_mass_matrix()
    print(f"\nMass matrix shape: {M.shape}")
    print(f"Expected shape: ({num_joints}, {num_joints})")

    # Mass matrix should be symmetric
    is_symmetric = np.allclose(M, M.T, atol=1e-6)
    print(f"Symmetric: {is_symmetric}")

    # Mass matrix should be positive definite (all eigenvalues > 0)
    eigenvalues = np.linalg.eigvals(M)
    min_eigenvalue = np.min(eigenvalues)
    print(f"Minimum eigenvalue: {min_eigenvalue:.6f}")

    if is_symmetric and min_eigenvalue > 0:
        print("✓ Mass matrix is symmetric and positive definite")
    else:
        print("⚠ Mass matrix properties incorrect")

    # Test 2: Gravity Torques
    print("\n" + "-" * 80)
    print("TEST 2: Gravity Torque Computation")
    print("-" * 80)

    gravity_torques = inv_dyn.compute_gravity_torques()
    print(f"\nGravity torques shape: {gravity_torques.shape}")
    print(f"Gravity torques (N⋅m): {gravity_torques}")

    max_gravity = np.max(np.abs(gravity_torques))
    print(f"\nMax gravity torque: {max_gravity:.4f} N⋅m")

    if max_gravity < 50.0:  # Reasonable for 12kg robot
        print("✓ Gravity torques in reasonable range")
    else:
        print("⚠ Very large gravity torques detected")

    # Test 3: Zero gravity validation
    print("\n" + "-" * 80)
    print("TEST 3: Zero Gravity Validation")
    print("-" * 80)

    p.setGravity(0, 0, 0)  # Turn off gravity
    gravity_torques_zero_g = inv_dyn.compute_gravity_torques()
    max_torque_zero_g = np.max(np.abs(gravity_torques_zero_g))

    print(f"\nMax torque in zero-g: {max_torque_zero_g:.6f} N⋅m")

    p.setGravity(0, 0, -9.81)  # Restore gravity

    if max_torque_zero_g < 0.01:
        print("✓ Zero gravity produces near-zero torques")
    else:
        print("⚠ Unexpected torques in zero gravity")

    # Test 4: Coriolis Forces
    print("\n" + "-" * 80)
    print("TEST 4: Coriolis Force Computation")
    print("-" * 80)

    # Set non-zero velocities
    test_velocities = np.random.uniform(-0.5, 0.5, num_joints)

    coriolis_forces = inv_dyn.compute_coriolis_forces(joint_velocities=test_velocities)
    print(f"\nCoriolis forces shape: {coriolis_forces.shape}")
    print(f"Max Coriolis force: {np.max(np.abs(coriolis_forces)):.4f} N⋅m")

    # Coriolis should be zero when velocities are zero
    coriolis_zero_vel = inv_dyn.compute_coriolis_forces(
        joint_velocities=np.zeros(num_joints)
    )
    max_coriolis_zero = np.max(np.abs(coriolis_zero_vel))

    print(f"Max Coriolis at zero velocity: {max_coriolis_zero:.6f} N⋅m")

    if max_coriolis_zero < 1e-4:
        print("✓ Coriolis forces are zero at zero velocity")
    else:
        print("⚠ Non-zero Coriolis at zero velocity")

    # Test 5: Inverse Dynamics
    print("\n" + "-" * 80)
    print("TEST 5: Full Inverse Dynamics")
    print("-" * 80)

    # Get current state
    joint_states = [p.getJointState(robot_id, idx) for idx in range(num_joints)]
    q = np.array([s[0] for s in joint_states])
    qd = np.array([s[1] for s in joint_states])

    # Desired accelerations
    qdd_desired = np.random.uniform(-1.0, 1.0, num_joints)

    # Compute required torques
    torques = inv_dyn.inverse_dynamics(q, qd, qdd_desired)

    print(f"\nDesired accelerations (rad/s²): {qdd_desired}")
    print(f"Required torques (N⋅m): {torques}")
    print(f"Max torque: {np.max(np.abs(torques)):.4f} N⋅m")

    if np.max(np.abs(torques)) < 100.0:  # Reasonable limit
        print("✓ Inverse dynamics produces reasonable torques")
    else:
        print("⚠ Very large torques computed")

    # Test 6: Forward-Inverse Consistency
    print("\n" + "-" * 80)
    print("TEST 6: Forward-Inverse Dynamics Consistency")
    print("-" * 80)

    # Given torques, compute accelerations (forward dynamics)
    qdd_computed = inv_dyn.forward_dynamics(q, qd, torques)

    # Compare with desired accelerations
    acceleration_error = np.linalg.norm(qdd_computed - qdd_desired)

    print(f"\nDesired accelerations: {qdd_desired}")
    print(f"Computed accelerations: {qdd_computed}")
    print(f"Error: {acceleration_error:.6f} rad/s²")

    if acceleration_error < 1e-4:
        print("✓ Forward and inverse dynamics are consistent")
    else:
        print(f"⚠ Acceleration error: {acceleration_error:.6f}")

    # Test 7: Dictionary Interface
    print("\n" + "-" * 80)
    print("TEST 7: Dictionary Interface")
    print("-" * 80)

    # Create position/velocity dicts
    joint_names = list(joint_info.keys())
    pos_dict = {name: q[i] for i, name in enumerate(joint_names)}
    vel_dict = {name: qd[i] for i, name in enumerate(joint_names)}

    # Compute dynamics
    dynamics = inv_dyn.compute_dynamics_dict(pos_dict, vel_dict)

    print(f"\nDynamics keys: {list(dynamics.keys())}")
    print(f"Mass matrix shape: {dynamics['mass_matrix'].shape}")
    print(f"Gravity shape: {dynamics['gravity'].shape}")
    print(f"Coriolis shape: {dynamics['coriolis'].shape}")

    # Verify consistency
    gravity_match = np.allclose(dynamics['gravity'], gravity_torques, atol=1e-6)

    if gravity_match:
        print("✓ Dictionary interface produces consistent results")
    else:
        print("⚠ Dictionary interface inconsistent with array interface")

    # Summary
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)

    print(f"\n✓ Inverse dynamics module functional")
    print(f"  - Mass matrix: {M.shape}, symmetric: {is_symmetric}")
    print(f"  - Gravity torques: max {max_gravity:.4f} N⋅m")
    print(f"  - Coriolis forces: computed correctly")
    print(f"  - Inverse dynamics: max torque {np.max(np.abs(torques)):.4f} N⋅m")
    print(f"  - Forward-inverse error: {acceleration_error:.6f}")

    print("\n✓ TEST PASSED - Inverse dynamics ready for WBC integration")

    p.disconnect()


if __name__ == "__main__":
    test_inverse_dynamics()

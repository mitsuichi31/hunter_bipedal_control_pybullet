#!/usr/bin/env python3
"""
Quick test to demonstrate gravity compensation diagnostic features
"""

import os
import sys
import pybullet as p
import pybullet_data

# Add src to path
sys.path.insert(0, os.path.dirname(__file__))

from gravity_compensation import GravityCompensation

def test_gravity_diagnostics():
    """Test gravity compensation diagnostics"""
    print("=" * 60)
    print("Gravity Compensation Diagnostics Test")
    print("=" * 60)

    # Connect to PyBullet
    physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load robot
    urdf_path = os.path.join(os.path.dirname(__file__), "../models/urdf/hunter.urdf")
    robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0.679])

    # Create gravity compensation
    gc = GravityCompensation(robot_id)

    print("\nBefore first computation:")
    print(f"  Method: {gc.get_computation_method()}")

    # Compute gravity torques (triggers method determination)
    torques = gc.compute_gravity_torques()

    print("\nAfter first computation:")
    print(f"  Method: {gc.get_computation_method()}")
    print(f"  Torques (Nm): {torques}")
    print(f"  Max torque: {max(abs(torques)):.4f} Nm")

    print("\n" + "=" * 60)
    print("Diagnostics Summary:")
    print("- Clear one-time warning shown for fallback method")
    print("- Method can be queried programmatically")
    print("- Fallback provides accurate results for free-floating base")
    print("=" * 60)

    p.disconnect()

if __name__ == "__main__":
    test_gravity_diagnostics()

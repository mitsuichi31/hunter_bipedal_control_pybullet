#!/usr/bin/env python3
"""
Find a stable standing configuration for Hunter robot
Strategy: Place feet on ground, adjust base height accordingly
"""

import pybullet as p
import pybullet_data
import numpy as np

from robot_constants import BASE_HEIGHT, STANDING_CONFIG


def test_configuration(config, base_height):
    """Test if a configuration is stable"""

    joint_dict = {}
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        if info[2] == p.JOINT_REVOLUTE:
            joint_dict[info[1].decode('utf-8')] = i

    # Reset base
    p.resetBasePositionAndOrientation(robot_id, [0, 0, base_height], [0, 0, 0, 1])

    # Set joint angles
    for name, angle in config.items():
        if name in joint_dict:
            p.resetJointState(robot_id, joint_dict[name], angle, 0.0)

    # Get foot positions
    left_foot = p.getLinkState(robot_id, joint_dict['leg_l5_joint'])[0]
    right_foot = p.getLinkState(robot_id, joint_dict['leg_r5_joint'])[0]
    base_pos = p.getBasePositionAndOrientation(robot_id)[0]

    foot_diff = abs(left_foot[2] - right_foot[2])
    avg_foot_height = (left_foot[2] + right_foot[2]) / 2

    return {
        'base_z': base_pos[2],
        'left_foot_z': left_foot[2],
        'right_foot_z': right_foot[2],
        'foot_asymmetry': foot_diff,
        'avg_foot_height': avg_foot_height,
        'left_foot_xy': left_foot[0:2],
        'right_foot_xy': right_foot[0:2],
    }


# Connect
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load robot
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.join(script_dir, '../models/urdf/hunter.urdf')
robot_id = p.loadURDF(urdf_path, [0, 0, BASE_HEIGHT])

print("=" * 70)
print("Finding Stable Standing Configuration for Hunter Robot")
print("=" * 70)

# Test different configurations
# For a 5-DOF leg: hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch

configs_to_test = [
    {
        'name': 'Original (broken)',
        'config': {
            'leg_l1_joint': -0.1, 'leg_l2_joint': 0.0, 'leg_l3_joint': -0.4,
            'leg_l4_joint': 0.8, 'leg_l5_joint': -0.4,
            'leg_r1_joint': 0.1, 'leg_r2_joint': 0.0, 'leg_r3_joint': -0.4,
            'leg_r4_joint': 0.8, 'leg_r5_joint': -0.4,
        },
        'base_height': 0.4
    },
    {
        'name': 'Symmetric straight legs',
        'config': STANDING_CONFIG,
        'base_height': BASE_HEIGHT
    },
    {
        'name': 'Small knee bend',
        'config': {
            'leg_l1_joint': -0.1, 'leg_l2_joint': 0.0, 'leg_l3_joint': -0.2,
            'leg_l4_joint': 0.4, 'leg_l5_joint': -0.2,
            'leg_r1_joint': 0.1, 'leg_r2_joint': 0.0, 'leg_r3_joint': -0.2,
            'leg_r4_joint': 0.4, 'leg_r5_joint': -0.2,
        },
        'base_height': 0.4
    },
]

best_config = None
min_asymmetry = float('inf')

for test in configs_to_test:
    print(f"\nTesting: {test['name']}")
    print("-" * 70)

    result = test_configuration(test['config'], test['base_height'])

    print(f"  Base height:        {result['base_z']:.3f} m")
    print(f"  Left foot z:        {result['left_foot_z']:.3f} m")
    print(f"  Right foot z:       {result['right_foot_z']:.3f} m")
    print(f"  Foot asymmetry:     {result['foot_asymmetry']*1000:.1f} mm")
    print(f"  Avg foot height:    {result['avg_foot_height']:.3f} m")

    # Calculate needed base height for feet on ground (z=0)
    needed_height = test['base_height'] - result['avg_foot_height']
    print(f"  → Need base height: {needed_height:.3f} m for feet on ground")

    # Test with corrected height
    result2 = test_configuration(test['config'], needed_height)
    print(f"  After adjustment:")
    print(f"    Base z:           {result2['base_z']:.3f} m")
    print(f"    Left foot z:      {result2['left_foot_z']:.3f} m")
    print(f"    Right foot z:     {result2['right_foot_z']:.3f} m")
    print(f"    Asymmetry:        {result2['foot_asymmetry']*1000:.1f} mm")

    if result2['foot_asymmetry'] < min_asymmetry:
        min_asymmetry = result2['foot_asymmetry']
        best_config = {
            'name': test['name'],
            'config': test['config'],
            'base_height': needed_height,
            'result': result2
        }

print("\n" + "=" * 70)
print("RECOMMENDED CONFIGURATION")
print("=" * 70)
print(f"\nName: {best_config['name']}")
print(f"Base height: {best_config['base_height']:.3f} m")
print(f"Foot asymmetry: {best_config['result']['foot_asymmetry']*1000:.1f} mm")
print("\nJoint angles (copy to code):")
print("standing_config = {")
for joint, angle in best_config['config'].items():
    print(f"    '{joint}': {angle:.3f},")
print("}")
print(f"\n# Use with: basePosition=[0, 0, {best_config['base_height']:.3f}]")

p.disconnect()

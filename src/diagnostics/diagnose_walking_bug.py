#!/usr/bin/env python3
"""
Simple diagnostic to identify the walking mode coordinate frame bug
"""

import sys
import os
import numpy as np

# Add src to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gait_generator import GaitGenerator, GaitParams


def analyze_gait_trajectories():
    """Analyze what the gait generator produces"""
    print("=" * 70)
    print("GAIT GENERATOR TRAJECTORY ANALYSIS")
    print("=" * 70)

    gait_params = GaitParams(
        step_length=0.08,
        step_height=0.04,
        step_period=1.2,
        stance_width=0.18,
        body_height=0.55
    )

    gait = GaitGenerator(gait_params)

    print("\nGait trajectories over 3 seconds:")
    print(f"{'Time(s)':>7} | {'Left X':>8} {'Left Y':>8} {'Left Z':>8} | {'Right X':>8} {'Right Y':>8} {'Right Z':>8}")
    print("-" * 80)

    for i in range(31):  # 0 to 3 seconds
        t = i * 0.1
        left_pos, right_pos = gait.get_foot_trajectories(t)

        print(f"{t:7.1f} | "
              f"{left_pos[0]:8.4f} {left_pos[1]:8.4f} {left_pos[2]:8.4f} | "
              f"{right_pos[0]:8.4f} {right_pos[1]:8.4f} {right_pos[2]:8.4f}")

    print("\n" + "=" * 70)
    print("OBSERVATIONS:")
    print("=" * 70)
    print()
    print("1. Foot X-coordinates oscillate around 0, ranging from about -0.04 to +0.04")
    print("   This is RELATIVE movement - feet stepping forward/back")
    print()
    print("2. Foot Y-coordinates are CONSTANT at ±0.09 (stance width / 2)")
    print("   This is the lateral offset of each foot")
    print()
    print("3. Foot Z-coordinates alternate: swing phase lifts to ~0.04m, stance at 0m")
    print()
    print("=" * 70)
    print("THE BUG:")
    print("=" * 70)
    print()
    print("In main_simulation.py, lines 104-113:")
    print()
    print("    left_target_world = np.array([")
    print("        base_pos[0] + left_target[0],  # <-- THIS IS WRONG!")
    print("        left_target[1],")
    print("        left_target[2]")
    print("    ])")
    print()
    print("PROBLEM:")
    print("  - Gait generator outputs RELATIVE foot positions (oscillating around 0)")
    print("  - Code adds base_pos[0] (current robot x-position)")
    print("  - As robot moves forward, base_pos[0] increases")
    print("  - This creates POSITIVE FEEDBACK: feet keep moving forward!")
    print()
    print("SCENARIO:")
    print("  t=0.0s: base_pos[0]=0.0,  foot_target_x=0.02  → IK target: x=0.02")
    print("  t=0.1s: base_pos[0]=0.5,  foot_target_x=0.04  → IK target: x=0.54  (!)")
    print("  t=0.2s: base_pos[0]=2.0,  foot_target_x=0.02  → IK target: x=2.02 (!)")
    print("  t=0.3s: base_pos[0]=10.0, foot_target_x=0.0   → IK target: x=10.0 (!)")
    print()
    print("The feet are constantly commanded to move forward relative to the")
    print("accelerating base, causing the robot to 'fly' through space.")
    print()
    print("=" * 70)
    print("THE FIX:")
    print("=" * 70)
    print()
    print("OPTION 1: Gait generator already outputs world coordinates")
    print("  - Remove the base_pos[0] offset entirely")
    print("  - Use: left_target_world = left_target  # Direct copy")
    print()
    print("OPTION 2: Gait generator outputs body-relative coordinates")
    print("  - Keep current approach BUT fix gait generator to output")
    print("    absolute world coordinates that account for forward progress")
    print()
    print("ANALYSIS: Looking at gait_generator.py, the trajectories ARE")
    print("body-relative (oscillating around 0). BUT the current implementation")
    print("uses DYNAMIC base position which creates positive feedback.")
    print()
    print("CORRECT FIX:")
    print("  Track a 'reference x position' that advances steadily:")
    print()
    print("  left_target_world = np.array([")
    print("      self.reference_x + left_target[0],  # Use steady reference")
    print("      left_target[1],")
    print("      left_target[2]")
    print("  ])")
    print()
    print("  Where reference_x advances based on desired walking speed,")
    print("  NOT the actual robot position.")
    print("=" * 70)


if __name__ == "__main__":
    analyze_gait_trajectories()

#!/usr/bin/env python3
"""
Detailed walking mode test with diagnostics
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from simulation_env import HunterSimulation
from inverse_kinematics import BipedalIKSolver
from gait_generator import GaitGenerator, GaitParams
from pd_controller import MultiJointPDController


def test_walking_with_diagnostics():
    """Test walking mode with detailed diagnostics"""
    print("=" * 70)
    print("WALKING MODE - DETAILED DIAGNOSTICS")
    print("=" * 70)

    # Setup
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

    sim = HunterSimulation(urdf_path=urdf_path, dt=0.001, use_gui=False)
    sim.connect()
    sim.load_robot(start_position=[0, 0, 0.679])

    # Create components
    ik_solver = BipedalIKSolver(sim.robot_id, sim.joint_dict)
    gait_params = GaitParams(
        step_length=0.08,
        step_height=0.04,
        step_period=1.2,
        stance_width=0.18,
        body_height=0.55
    )
    gait = GaitGenerator(gait_params)
    pd_controller = MultiJointPDController(default_kp=200.0, default_kd=20.0)

    # Tracking
    sim_time = 0.0
    reference_x = 0.0
    dt = 0.001
    control_dt = 0.01  # Control at 100 Hz

    print(f"\nParameters:")
    print(f"  Step length: {gait_params.step_length} m")
    print(f"  Step period: {gait_params.step_period} s")
    print(f"  Desired velocity: {gait_params.step_length / gait_params.step_period:.3f} m/s")
    print(f"  Simulation dt: {dt} s")
    print(f"  Control dt: {control_dt} s")

    print(f"\n{'Time':>6} | {'RefX':>8} {'BaseX':>8} {'BaseZ':>8} | {'L_tgt_X':>8} {'R_tgt_X':>8} | {'L_act_X':>8} {'R_act_X':>8} | Status")
    print("-" * 120)

    control_step = 0
    for i in range(int(3.0 / dt)):  # 3 seconds
        sim_time = i * dt

        # Control update (every control_dt)
        if i % int(control_dt / dt) == 0:
            # Get gait trajectories
            left_target_body, right_target_body = gait.get_foot_trajectories(sim_time)

            # Convert to world coordinates using reference_x
            left_target_world = np.array([
                reference_x + left_target_body[0],
                left_target_body[1],
                left_target_body[2]
            ])
            right_target_world = np.array([
                reference_x + right_target_body[0],
                right_target_body[1],
                right_target_body[2]
            ])

            # Solve IK
            joint_angles = ik_solver.solve_both_legs(
                list(left_target_world),
                list(right_target_world)
            )

            if joint_angles:
                joint_states = sim.get_joint_states()
                current_pos = {name: state[0] for name, state in joint_states.items()}
                current_vel = {name: state[1] for name, state in joint_states.items()}

                torques = pd_controller.compute_torques(
                    target_positions=joint_angles,
                    current_positions=current_pos,
                    current_velocities=current_vel
                )

                sim.set_joint_torques(torques)

            # Update reference position
            forward_velocity = gait_params.step_length / gait_params.step_period
            reference_x += forward_velocity * control_dt

            # Diagnostics (every 0.1 seconds)
            if control_step % 10 == 0:
                base_pos, _, _, _ = sim.get_base_state()
                left_foot_actual, right_foot_actual = ik_solver.get_foot_positions()

                status = "OK"
                if base_pos[2] > 1.5:
                    status = "FLYING!"
                elif base_pos[2] < 0.5:
                    status = "FALLEN"

                print(f"{sim_time:6.2f} | "
                      f"{reference_x:8.3f} {base_pos[0]:8.3f} {base_pos[2]:8.3f} | "
                      f"{left_target_world[0]:8.3f} {right_target_world[0]:8.3f} | "
                      f"{left_foot_actual[0]:8.3f} {right_foot_actual[0]:8.3f} | "
                      f"{status}")

            control_step += 1

        # Simulation step
        sim.step()

    # Final stats
    final_pos, _, _, _ = sim.get_base_state()
    print("\n" + "=" * 70)
    print("RESULTS:")
    print("=" * 70)
    print(f"Final position: [{final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f}]")
    print(f"Expected x-travel: {3.0 * gait_params.step_length / gait_params.step_period:.2f} m")
    print(f"Actual x-travel: {final_pos[0]:.2f} m")
    print(f"Final reference_x: {reference_x:.2f} m")

    if final_pos[2] > 1.5:
        print("❌ Robot is flying!")
    elif final_pos[2] < 0.5:
        print("❌ Robot has fallen!")
    elif abs(final_pos[0]) > 1.0:
        print("❌ Robot traveled too far!")
    else:
        print("✓ Robot stayed grounded!")

    sim.disconnect()


if __name__ == "__main__":
    test_walking_with_diagnostics()

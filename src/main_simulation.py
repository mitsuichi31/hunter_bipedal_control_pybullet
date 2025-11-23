"""
Main simulation loop for Hunter bipedal robot walking

This script integrates:
- PyBullet simulation environment
- PD control
- Inverse kinematics
- Gait generation
"""

import os
import sys
import time
import numpy as np
import argparse
import pybullet as p

from simulation_env import HunterSimulation
from pd_controller import MultiJointPDController
from inverse_kinematics import BipedalIKSolver
from gait_generator import GaitGenerator, GaitParams, StaticGaitGenerator
from balance_controller import ZMPBalanceController, BalanceParams
from mpc_controller import MPCParams
from mpc_wbc_controller import MPCWBCController
from wbc_controller import WBCParams
from wbc_walking_controller import WBCWalkingController, WBCWalkingParams


class WalkingController:
    """
    Main walking controller that integrates all components
    """

    def __init__(self,
                 sim: HunterSimulation,
                 gait_params: GaitParams = None,
                 pd_kp: float = 200.0,
                 pd_kd: float = 20.0):
        """
        Initialize walking controller

        Args:
            sim: Simulation environment
            gait_params: Gait parameters
            pd_kp: PD controller proportional gain
            pd_kd: PD controller derivative gain
        """
        self.sim = sim

        # Create IK solver
        self.ik_solver = BipedalIKSolver(sim.robot_id, sim.joint_dict)

        # Create PD controller
        self.pd_controller = MultiJointPDController(default_kp=pd_kp, default_kd=pd_kd)

        # Set different gains for different joints
        # Hip joints (leg_l1, leg_r1) - lower gain
        self.pd_controller.set_joint_gains("leg_l1_joint", kp=150.0, kd=15.0)
        self.pd_controller.set_joint_gains("leg_r1_joint", kp=150.0, kd=15.0)

        # Yaw joints (leg_l2, leg_r2) - medium gain
        self.pd_controller.set_joint_gains("leg_l2_joint", kp=180.0, kd=18.0)
        self.pd_controller.set_joint_gains("leg_r2_joint", kp=180.0, kd=18.0)

        # Knee joints (leg_l3, leg_r3, leg_l4, leg_r4) - higher gain
        self.pd_controller.set_joint_gains("leg_l3_joint", kp=250.0, kd=25.0)
        self.pd_controller.set_joint_gains("leg_r3_joint", kp=250.0, kd=25.0)
        self.pd_controller.set_joint_gains("leg_l4_joint", kp=250.0, kd=25.0)
        self.pd_controller.set_joint_gains("leg_r4_joint", kp=250.0, kd=25.0)

        # Ankle joints (leg_l5, leg_r5) - medium gain
        self.pd_controller.set_joint_gains("leg_l5_joint", kp=180.0, kd=18.0)
        self.pd_controller.set_joint_gains("leg_r5_joint", kp=180.0, kd=18.0)

        # Create gait generator
        self.gait_generator = GaitGenerator(gait_params)

        # State
        self.time = 0.0
        self.reference_x = 0.0  # Reference x-position for foot placement

    def get_base_position_in_foot_frame(self) -> np.ndarray:
        """Get base position relative to average foot position"""
        left_foot_pos, right_foot_pos = self.ik_solver.get_foot_positions()
        avg_foot_pos = (left_foot_pos + right_foot_pos) / 2.0

        base_pos, _, _, _ = self.sim.get_base_state()

        return base_pos - avg_foot_pos

    def control_step(self, dt: float):
        """
        Execute one control step

        Args:
            dt: Time step
        """
        # Get current base position
        base_pos, base_orn, _, _ = self.sim.get_base_state()

        # Generate foot trajectories (body-relative coordinates)
        left_target, right_target = self.gait_generator.get_foot_trajectories(self.time)

        # Convert from body-relative to world coordinates
        # Use reference_x (steady forward progress) instead of base_pos[0] (actual position)
        # to avoid positive feedback loop
        left_target_world = np.array([
            self.reference_x + left_target[0],
            left_target[1],  # y is lateral offset (already in world coords)
            left_target[2]   # z is height above ground
        ])
        right_target_world = np.array([
            self.reference_x + right_target[0],
            right_target[1],
            right_target[2]
        ])

        # Solve IK for both legs
        target_joint_positions = self.ik_solver.solve_both_legs(
            left_target=left_target_world,
            right_target=right_target_world
        )

        if not target_joint_positions:
            print("Warning: IK solution failed")
            return

        # Get current joint states
        joint_states = self.sim.get_joint_states()

        current_positions = {}
        current_velocities = {}

        for joint_name, state in joint_states.items():
            current_positions[joint_name] = state[0]  # position
            current_velocities[joint_name] = state[1]  # velocity

        # Compute control torques using PD controller
        torques = self.pd_controller.compute_torques(
            target_positions=target_joint_positions,
            current_positions=current_positions,
            current_velocities=current_velocities
        )

        # Apply torques
        self.sim.set_joint_torques(torques)

        # Update time and reference position
        self.time += dt
        # Update reference x-position based on desired forward velocity
        # Average forward velocity = step_length / step_period
        if self.gait_generator.params is not None:
            forward_velocity = self.gait_generator.params.step_length / self.gait_generator.params.step_period
            self.reference_x += forward_velocity * dt

    def reset(self):
        """Reset controller state"""
        self.time = 0.0
        self.reference_x = 0.0
        self.gait_generator.reset()


def run_standing_test_mpc(duration: float = 10.0, use_gui: bool = True):
    """
    Test robot standing using MPC+ZMP balance control

    Args:
        duration: Test duration in seconds
        use_gui: Whether to use GUI
    """
    print("Running standing test with MPC+ZMP control...")

    # Get URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

    # Create simulation
    sim = HunterSimulation(urdf_path=urdf_path, dt=0.001, use_gui=use_gui)
    sim.connect()
    sim.load_robot(start_position=[0, 0, 0.679])

    # Create balance controller with MPC
    balance_params = BalanceParams(
        mpc_dt=0.01,  # 10ms MPC update rate (faster response)
        com_height=0.55,  # Adjusted for new base height
        stance_width=0.18
    )

    mpc_params = MPCParams(
        prediction_horizon=15,
        control_horizon=8,
        dt=0.01,
        Q_position=10.0,      # Higher position tracking weight
        Q_velocity=1.0,       # Higher velocity tracking weight
        R_zmp=1e-6            # Lower control effort penalty
    )

    balance_controller = ZMPBalanceController(
        robot_id=sim.robot_id,
        joint_dict=sim.joint_dict,
        balance_params=balance_params,
        mpc_params=mpc_params
    )

    # Create PD controller for joint tracking
    pd_controller = MultiJointPDController(default_kp=200.0, default_kd=20.0)

    # Set higher gains for critical joints
    pd_controller.set_joint_gains("leg_l3_joint", kp=300.0, kd=30.0)
    pd_controller.set_joint_gains("leg_r3_joint", kp=300.0, kd=30.0)
    pd_controller.set_joint_gains("leg_l4_joint", kp=300.0, kd=30.0)
    pd_controller.set_joint_gains("leg_r4_joint", kp=300.0, kd=30.0)

    print("Initializing with MPC balance control...")

    # Get initial balanced configuration
    target_positions = balance_controller.compute_standing_balance()

    print("Target joint positions (MPC-based):")
    for joint_name, angle in target_positions.items():
        print(f"  {joint_name}: {angle:.3f} rad ({np.degrees(angle):.1f} deg)")

    # Disable default motors
    print("Disabling default motors...")
    for joint_name in target_positions.keys():
        joint_idx = sim.get_joint_index(joint_name)
        if joint_idx is not None:
            p.setJointMotorControl2(
                bodyIndex=sim.robot_id,
                jointIndex=joint_idx,
                controlMode=p.VELOCITY_CONTROL,
                force=0.0
            )

    # Set initial joint positions
    sim.reset_robot(position=[0, 0, 0.679], joint_positions=target_positions)

    # Enable position control
    print("Enabling position control with MPC updates...")
    for joint_name, target_angle in target_positions.items():
        joint_idx = sim.get_joint_index(joint_name)
        if joint_idx is not None:
            if 'l3' in joint_name or 'r3' in joint_name or 'l4' in joint_name or 'r4' in joint_name:
                max_force = 500.0
            else:
                max_force = 300.0

            p.setJointMotorControl2(
                bodyIndex=sim.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angle,
                force=max_force
            )

    print(f"Running MPC-controlled simulation for {duration} seconds...")

    start_time = time.time()
    sim_time = 0.0
    mpc_update_interval = 0.01  # Update MPC every 10ms (matching balance_params)

    # Get initial base position
    initial_pos, initial_orn, _, _ = sim.get_base_state()
    initial_euler = p.getEulerFromQuaternion(initial_orn)
    print(f"Initial base height: {initial_pos[2]:.3f} m")
    print(f"Initial orientation (roll, pitch, yaw): ({np.degrees(initial_euler[0]):.1f}°, {np.degrees(initial_euler[1]):.1f}°, {np.degrees(initial_euler[2]):.1f}°)")

    last_mpc_update = 0.0

    while sim_time < duration:
        # Update MPC at specified rate
        if sim_time - last_mpc_update >= mpc_update_interval:
            # Compute new balanced configuration
            target_positions = balance_controller.compute_standing_balance()

            # Update position control targets
            for joint_name, target_angle in target_positions.items():
                joint_idx = sim.get_joint_index(joint_name)
                if joint_idx is not None:
                    if 'l3' in joint_name or 'r3' in joint_name or 'l4' in joint_name or 'r4' in joint_name:
                        max_force = 500.0
                    else:
                        max_force = 300.0

                    p.setJointMotorControl2(
                        bodyIndex=sim.robot_id,
                        jointIndex=joint_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_angle,
                        force=max_force
                    )

            last_mpc_update = sim_time

        # Step simulation
        sim.step()
        sim_time += sim.dt

        # Real-time visualization
        if use_gui:
            elapsed = time.time() - start_time
            if sim_time > elapsed:
                time.sleep(sim_time - elapsed)

        # Print status every second
        if int(sim_time) % 1 == 0 and sim_time > 0:
            if abs(sim_time - int(sim_time)) < 0.001:
                base_pos, base_orn, _, _ = sim.get_base_state()
                euler = p.getEulerFromQuaternion(base_orn)
                print(f"Time: {sim_time:.1f}s, Height: {base_pos[2]:.3f} m, " +
                      f"Orient(R/P/Y): ({np.degrees(euler[0]):6.1f}°, {np.degrees(euler[1]):6.1f}°, {np.degrees(euler[2]):6.1f}°)")

    # Get final base position and orientation
    final_pos, final_orn, _, _ = sim.get_base_state()
    final_euler = p.getEulerFromQuaternion(final_orn)

    print(f"\n{'='*60}")
    print(f"Final State:")
    print(f"{'='*60}")
    print(f"Position: [{final_pos[0]:7.3f}, {final_pos[1]:7.3f}, {final_pos[2]:7.3f}] m")
    print(f"Orientation (Roll/Pitch/Yaw):")
    print(f"  Roll:  {np.degrees(final_euler[0]):7.1f}° (should be ~0° for upright)")
    print(f"  Pitch: {np.degrees(final_euler[1]):7.1f}° (should be ~0° for upright)")
    print(f"  Yaw:   {np.degrees(final_euler[2]):7.1f}°")

    # Check if robot is upright
    roll_deg = abs(np.degrees(final_euler[0]))
    pitch_deg = abs(np.degrees(final_euler[1]))

    print(f"\n{'='*60}")
    if roll_deg > 45 or pitch_deg > 45:
        print(f"✗ Robot has fallen over!")
        print(f"  Roll angle: {roll_deg:.1f}° (>45° indicates fallen)")
        print(f"  Pitch angle: {pitch_deg:.1f}° (>45° indicates fallen)")
    else:
        print(f"✓ Robot is upright!")
        print(f"  Roll angle: {roll_deg:.1f}° (within limits)")
        print(f"  Pitch angle: {pitch_deg:.1f}° (within limits)")
    print(f"{'='*60}")

    sim.disconnect()
    print("\nMPC standing test completed!")


def run_standing_test(duration: float = 10.0, use_gui: bool = True):
    """
    Test robot standing in place

    Args:
        duration: Test duration in seconds
        use_gui: Whether to use GUI
    """
    print("Running standing test (using manual joint positions)...")

    # Get URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

    # Create simulation
    sim = HunterSimulation(urdf_path=urdf_path, dt=0.001, use_gui=use_gui)
    sim.connect()
    # Start at appropriate height for standing (corrected from 0.40 to 0.679)
    sim.load_robot(start_position=[0, 0, 0.679])

    # Create PD controller
    pd_controller = MultiJointPDController(default_kp=200.0, default_kd=20.0)

    # Set higher gains for critical joints
    pd_controller.set_joint_gains("leg_l3_joint", kp=300.0, kd=30.0)
    pd_controller.set_joint_gains("leg_r3_joint", kp=300.0, kd=30.0)
    pd_controller.set_joint_gains("leg_l4_joint", kp=300.0, kd=30.0)
    pd_controller.set_joint_gains("leg_r4_joint", kp=300.0, kd=30.0)

    print("Initializing standing pose...")

    # Use stable standing configuration
    # Straight legs with symmetric hip roll (feet on ground at z=0)
    target_positions = {
        # Left leg - straight with slight outward roll
        'leg_l1_joint': -0.1,     # Hip roll - open left leg outward
        'leg_l2_joint': 0.0,      # Hip yaw - neutral
        'leg_l3_joint': 0.0,      # Hip pitch - straight
        'leg_l4_joint': 0.0,      # Knee - straight
        'leg_l5_joint': 0.0,      # Ankle - straight

        # Right leg - straight with slight outward roll
        'leg_r1_joint': 0.1,      # Hip roll - open right leg outward
        'leg_r2_joint': 0.0,      # Hip yaw - neutral
        'leg_r3_joint': 0.0,      # Hip pitch - straight
        'leg_r4_joint': 0.0,      # Knee - straight
        'leg_r5_joint': 0.0,      # Ankle - straight
    }

    print("Target joint positions (manual configuration):")
    for joint_name, angle in target_positions.items():
        print(f"  {joint_name}: {angle:.3f} rad ({np.degrees(angle):.1f} deg)")

    # IMPORTANT: Disable all default motors first
    # PyBullet enables motors by default which interferes with our control
    print("Disabling default motors...")
    for joint_name in target_positions.keys():
        joint_idx = sim.get_joint_index(joint_name)
        if joint_idx is not None:
            p.setJointMotorControl2(
                bodyIndex=sim.robot_id,
                jointIndex=joint_idx,
                controlMode=p.VELOCITY_CONTROL,
                force=0.0
            )

    # Set initial joint positions
    sim.reset_robot(position=[0, 0, 0.679], joint_positions=target_positions)

    # Enable position control
    print("Enabling position control motors...")
    for joint_name, target_angle in target_positions.items():
        joint_idx = sim.get_joint_index(joint_name)
        if joint_idx is not None:
            # Use default gains (work well in testing)
            # Just specify maximum force/torque
            if 'l3' in joint_name or 'r3' in joint_name or 'l4' in joint_name or 'r4' in joint_name:
                # Knee and hip pitch - critical for stability
                max_force = 500.0
            else:
                # Other joints
                max_force = 300.0

            p.setJointMotorControl2(
                bodyIndex=sim.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angle,
                force=max_force
            )

    print(f"Running simulation for {duration} seconds...")

    start_time = time.time()
    sim_time = 0.0

    # Get initial base position
    initial_pos, initial_orn, _, _ = sim.get_base_state()
    initial_euler = p.getEulerFromQuaternion(initial_orn)
    print(f"Initial base height: {initial_pos[2]:.3f} m")
    print(f"Initial orientation (roll, pitch, yaw): ({np.degrees(initial_euler[0]):.1f}°, {np.degrees(initial_euler[1]):.1f}°, {np.degrees(initial_euler[2]):.1f}°)")

    while sim_time < duration:
        # Step simulation
        sim.step()
        sim_time += sim.dt

        # Real-time visualization
        if use_gui:
            elapsed = time.time() - start_time
            if sim_time > elapsed:
                time.sleep(sim_time - elapsed)

        # Print status every second
        if int(sim_time) % 1 == 0 and sim_time > 0:
            if abs(sim_time - int(sim_time)) < 0.001:
                base_pos, base_orn, _, _ = sim.get_base_state()
                euler = p.getEulerFromQuaternion(base_orn)
                print(f"Time: {sim_time:.1f}s, Height: {base_pos[2]:.3f} m, " +
                      f"Orient(R/P/Y): ({np.degrees(euler[0]):6.1f}°, {np.degrees(euler[1]):6.1f}°, {np.degrees(euler[2]):6.1f}°)")

    # Get final base position and orientation
    final_pos, final_orn, _, _ = sim.get_base_state()
    final_euler = p.getEulerFromQuaternion(final_orn)

    print(f"\n{'='*60}")
    print(f"Final State:")
    print(f"{'='*60}")
    print(f"Position: [{final_pos[0]:7.3f}, {final_pos[1]:7.3f}, {final_pos[2]:7.3f}] m")
    print(f"Orientation (Roll/Pitch/Yaw):")
    print(f"  Roll:  {np.degrees(final_euler[0]):7.1f}° (should be ~0° for upright)")
    print(f"  Pitch: {np.degrees(final_euler[1]):7.1f}° (should be ~0° for upright)")
    print(f"  Yaw:   {np.degrees(final_euler[2]):7.1f}°")

    # Check if robot is upright
    roll_deg = abs(np.degrees(final_euler[0]))
    pitch_deg = abs(np.degrees(final_euler[1]))

    print(f"\n{'='*60}")
    if roll_deg > 45 or pitch_deg > 45:
        print(f"✗ Robot has fallen over!")
        print(f"  Roll angle: {roll_deg:.1f}° (>45° indicates fallen)")
        print(f"  Pitch angle: {pitch_deg:.1f}° (>45° indicates fallen)")
    else:
        print(f"✓ Robot is upright!")
        print(f"  Roll angle: {roll_deg:.1f}° (within limits)")
        print(f"  Pitch angle: {pitch_deg:.1f}° (within limits)")
    print(f"{'='*60}")

    sim.disconnect()
    print("\nStanding test completed!")


def run_wbc_test(duration: float = 10.0, use_gui: bool = True):
    """
    Test robot with MPC + WBC Controller

    Args:
        duration: Test duration in seconds
        use_gui: Whether to use GUI
    """
    print("Running WBC test (MPC + Whole-Body Control)...")

    # Get URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

    # Create simulation
    sim = HunterSimulation(urdf_path=urdf_path, dt=0.001, use_gui=use_gui)
    sim.connect()
    sim.load_robot(start_position=[0, 0, 0.679])

    # Stable standing configuration (straight legs, feet on ground)
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

    # Disable default motors
    for joint_name in standing_config.keys():
        joint_idx = sim.get_joint_index(joint_name)
        if joint_idx is not None:
            p.setJointMotorControl2(
                bodyIndex=sim.robot_id,
                jointIndex=joint_idx,
                controlMode=p.VELOCITY_CONTROL,
                force=0.0
            )

    # Set initial positions
    sim.reset_robot(position=[0, 0, 0.679], joint_positions=standing_config)

    # Initialize MPC + WBC Controller
    print("Initializing MPC + WBC Controller...")

    mpc_params = MPCParams(
        prediction_horizon=20,
        control_horizon=10,
        dt=0.03,  # 30Hz control
        com_height=0.55,  # Adjusted for new base height (was 0.35)
        gravity=9.81,
        Q_position=1.0,
        Q_velocity=0.1,
        R_zmp=1e-6,
        max_zmp_offset=0.08
    )

    wbc_params = WBCParams(
        friction_coef=0.5,
        max_normal_force=500.0,
        min_normal_force=1.0,
        w_force_tracking=1.0,
        w_force_regularization=0.01
    )

    controller = MPCWBCController(
        robot_id=sim.robot_id,
        joint_dict=sim.joint_dict,
        mpc_params=mpc_params,
        wbc_params=wbc_params
    )

    print("Controller initialized")
    print(f"Control frequency: {1.0/mpc_params.dt:.0f}Hz")
    print(f"MPC horizon: {mpc_params.prediction_horizon} steps")

    # Enable position control
    for joint_name, target_angle in standing_config.items():
        joint_idx = sim.get_joint_index(joint_name)
        if joint_idx is not None:
            p.setJointMotorControl2(
                bodyIndex=sim.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angle,
                force=100.0,
                maxVelocity=10.0
            )

    print(f"Running simulation for {duration}s...\n")

    sim_time = 0.0
    control_dt = mpc_params.dt
    last_control = 0.0

    while sim_time < duration:
        # Control update at MPC frequency
        if sim_time - last_control >= control_dt:
            try:
                # Update controller
                joint_commands = controller.update(control_dt)

                # Apply commands
                for joint_name, target_angle in joint_commands.items():
                    joint_idx = sim.get_joint_index(joint_name)
                    if joint_idx is not None:
                        p.setJointMotorControl2(
                            bodyIndex=sim.robot_id,
                            jointIndex=joint_idx,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=target_angle,
                            force=100.0,
                            maxVelocity=10.0
                        )

            except Exception as e:
                print(f"Controller error at t={sim_time:.3f}s: {e}")
                break

            last_control = sim_time

        # Step simulation
        p.stepSimulation()
        sim_time += sim.dt

        # Status output every 2 seconds
        if int(sim_time) % 2 == 0 and abs(sim_time - int(sim_time)) < 0.001:
            base_pos, base_orn, _, _ = sim.get_base_state()
            euler = p.getEulerFromQuaternion(base_orn)
            print(f"t={sim_time:4.1f}s: h={base_pos[2]:5.3f}m, "
                  f"R={np.degrees(euler[0]):6.1f}°, P={np.degrees(euler[1]):6.1f}°")

        if use_gui:
            time.sleep(0.001)

    # Final state
    final_pos, final_orn, _, _ = sim.get_base_state()
    final_euler = p.getEulerFromQuaternion(final_orn)

    roll_deg = abs(np.degrees(final_euler[0]))
    pitch_deg = abs(np.degrees(final_euler[1]))

    print(f"\n{'='*60}")
    print("Final State:")
    print(f"  Height: {final_pos[2]:.3f} m")
    print(f"  Roll:   {roll_deg:.1f}°")
    print(f"  Pitch:  {pitch_deg:.1f}°")

    if roll_deg < 15 and pitch_deg < 15:
        print("\n✓ SUCCESS! WBC controller achieved stable standing!")
    elif roll_deg < 30 and pitch_deg < 30:
        print("\n✓ GOOD! Minor tilt but standing")
    elif roll_deg < 45 and pitch_deg < 45:
        print("\n~ PARTIAL: Tilted but not fallen")
    else:
        print("\n✗ FAILED: Robot has fallen")
    print(f"{'='*60}")

    sim.disconnect()
    print("\nWBC test completed!")


def run_walking_simulation(duration: float = 20.0, use_gui: bool = True):
    """
    Run walking simulation (currently just maintains standing position)

    Note: Full walking implementation requires more sophisticated control.
    For now, this demonstrates stable standing using the straight-leg configuration.

    Args:
        duration: Simulation duration in seconds
        use_gui: Whether to use GUI
    """
    print("Running walking simulation...")
    print("Note: Currently maintains standing position (full walking in development)")

    # Get URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

    # Create simulation
    sim = HunterSimulation(urdf_path=urdf_path, dt=0.001, use_gui=use_gui)
    sim.connect()
    sim.load_robot(start_position=[0, 0, 0.679])

    # Use simple PD controller (same as standing mode)
    pd_controller = MultiJointPDController(default_kp=200.0, default_kd=20.0)

    # Set higher gains for critical joints (same as standing mode)
    pd_controller.set_joint_gains("leg_l3_joint", kp=300.0, kd=30.0)
    pd_controller.set_joint_gains("leg_r3_joint", kp=300.0, kd=30.0)
    pd_controller.set_joint_gains("leg_l4_joint", kp=300.0, kd=30.0)
    pd_controller.set_joint_gains("leg_r4_joint", kp=300.0, kd=30.0)

    # Target: straight legs with slight outward stance
    target_positions = {
        # Left leg
        'leg_l1_joint': -0.1,    # Hip roll - outward
        'leg_l2_joint': 0.0,     # Hip yaw
        'leg_l3_joint': 0.0,     # Hip pitch - straight
        'leg_l4_joint': 0.0,     # Knee - straight
        'leg_l5_joint': 0.0,     # Ankle - straight
        # Right leg
        'leg_r1_joint': 0.1,     # Hip roll - outward
        'leg_r2_joint': 0.0,     # Hip yaw
        'leg_r3_joint': 0.0,     # Hip pitch - straight
        'leg_r4_joint': 0.0,     # Knee - straight
        'leg_r5_joint': 0.0,     # Ankle - straight
    }

    print("Initializing standing pose...")

    # Initialize robot with target joint positions
    for joint_name, target_pos in target_positions.items():
        if joint_name in sim.joint_dict:
            joint_id = sim.joint_dict[joint_name]
            p.resetJointState(sim.robot_id, joint_id, target_pos)

    print(f"Running simulation for {duration} seconds...")
    print(f"Control: PD control (standing mode)")

    start_time = time.time()
    last_print_time = 0.0

    while sim.time < duration:
        # Get current joint states
        joint_states = sim.get_joint_states()

        current_positions = {}
        current_velocities = {}
        for joint_name, state in joint_states.items():
            current_positions[joint_name] = state[0]
            current_velocities[joint_name] = state[1]

        # Compute torques
        torques = pd_controller.compute_torques(
            target_positions=target_positions,
            current_positions=current_positions,
            current_velocities=current_velocities
        )

        # Apply torques
        sim.set_joint_torques(torques)

        # Step simulation
        sim.step()

        # Real-time visualization
        if use_gui:
            elapsed = time.time() - start_time
            if sim.time > elapsed:
                time.sleep(sim.time - elapsed)

        # Print progress (every 1 second)
        if sim.time - last_print_time >= 1.0:
            base_pos, base_orn, _, _ = sim.get_base_state()
            base_euler = p.getEulerFromQuaternion(base_orn)
            roll_deg = np.degrees(base_euler[0])
            pitch_deg = np.degrees(base_euler[1])

            print(f"Time: {sim.time:.1f}s | "
                  f"Pos: [{base_pos[0]:.2f}, {base_pos[1]:.2f}, {base_pos[2]:.2f}] | "
                  f"Roll: {roll_deg:.1f}° | Pitch: {pitch_deg:.1f}°")

            last_print_time = sim.time

    # Final assessment
    final_pos, final_orn, _, _ = sim.get_base_state()
    final_euler = p.getEulerFromQuaternion(final_orn)

    print(f"\nSimulation completed!")
    print(f"Final base position: [{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}]")
    print(f"Final orientation: Roll={np.degrees(final_euler[0]):.1f}°, Pitch={np.degrees(final_euler[1]):.1f}°")

    # Check if robot stayed upright
    if abs(np.degrees(final_euler[0])) < 15 and abs(np.degrees(final_euler[1])) < 15:
        print("✓ Robot remained upright!")
    else:
        print("✗ Robot fell")

    # Check if robot stayed on ground
    if 0.5 < final_pos[2] < 0.8:
        print("✓ Robot stayed on ground!")
    else:
        print(f"✗ Robot height issue: {final_pos[2]:.2f}m")

    print("\nNote: Walking mode architecture requires further development.")
    print("See WALKING_MODE_INVESTIGATION.md for technical details.")

    sim.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hunter bipedal robot walking simulation")
    parser.add_argument("--mode", type=str, default="standing",
                       choices=["standing", "standing-mpc", "wbc", "walking"],
                       help="Simulation mode")
    parser.add_argument("--duration", type=float, default=10.0,
                       help="Simulation duration (seconds)")
    parser.add_argument("--no-gui", action="store_true",
                       help="Disable GUI")

    args = parser.parse_args()

    use_gui = not args.no_gui

    if args.mode == "standing":
        run_standing_test(duration=args.duration, use_gui=use_gui)
    elif args.mode == "standing-mpc":
        run_standing_test_mpc(duration=args.duration, use_gui=use_gui)
    elif args.mode == "wbc":
        run_wbc_test(duration=args.duration, use_gui=use_gui)
    elif args.mode == "walking":
        run_walking_simulation(duration=args.duration, use_gui=use_gui)

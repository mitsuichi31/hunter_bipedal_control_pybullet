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
import random
import time
import numpy as np
import argparse
import pybullet as p

from robot_constants import BASE_HEIGHT, STANDING_CONFIG, get_stance_config, standing_config_copy
from simulation_env import HunterSimulation
from pd_controller import MultiJointPDController
from inverse_kinematics import BipedalIKSolver
from gait_generator import GaitGenerator, GaitParams, StaticGaitGenerator
from balance_controller import ZMPBalanceController, BalanceParams
from mpc_controller import MPCParams
from mpc_wbc_controller import MPCWBCController
from wbc_controller import WBCParams
from wbc_walking_controller import WBCWalkingController, WBCWalkingParams
from gait_generator import GaitParams
from com_planner_simple import SimpleCoMPlannerParams
from full_body_ik import FullBodyIKParams
from config_loader import load_gait_config, load_reference_config, load_task_config
from estimation.observer import Observer
from planning.centroidal_mpc import CentroidalMPC, CentroidalMPCConfig
from planning.gait_schedule import GaitSchedule
from planning.reference_manager import ReferenceTargets


def _build_physics_and_limits(task_config):
    """Helper to derive physics parameters and command limits from task config."""
    wbc_cfg = task_config.wbc if hasattr(task_config, "wbc") else {}
    physics_params = {}
    if "friction_coefficient" in wbc_cfg:
        physics_params["lateral_friction"] = wbc_cfg.get("friction_coefficient")
    for key_src in [
        "num_solver_iterations",
        "num_sub_steps",
        "erp",
        "contact_erp",
        "contact_cfm",
    ]:
        if key_src in wbc_cfg:
            physics_params[key_src] = wbc_cfg.get(key_src)

    command_limits = {
        "max_torque": wbc_cfg.get("max_torque"),
        "max_velocity": wbc_cfg.get("max_velocity"),
    }
    return physics_params, command_limits

# Phase 4: Position Control Walking System
from position_control_walking import PositionControlWalkingController, WalkingControllerParams


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

    task_config = load_task_config()
    physics_params, command_limits = _build_physics_and_limits(task_config)

    # Get URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

    # Create simulation
    sim = HunterSimulation(
        urdf_path=urdf_path,
        dt=0.001,
        use_gui=use_gui,
        physics_params=physics_params,
        command_limits=command_limits,
    )
    sim.connect(enable_stable_contacts=True)
    sim.load_robot(start_position=[0, 0, BASE_HEIGHT])
    sim.set_contact_properties(
        lateral_friction=physics_params.get("lateral_friction", 1.0),
        spinning_friction=0.2,
        rolling_friction=0.05,
        restitution=0.0,
    )

    observer = Observer()
    observer.reset()

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
    sim.reset_robot(position=[0, 0, BASE_HEIGHT], joint_positions=target_positions)

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
    initial_raw = sim.get_observations()
    initial_filtered = observer.update(
        base_pos=initial_raw["base_position"],
        base_vel=initial_raw["base_velocity"],
        joint_states=initial_raw["joint_states"],
        contact_forces=initial_raw["contact_forces"],
    )
    initial_pos = initial_filtered["base_position"]
    initial_orn = initial_raw["base_orientation"]
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

        raw_obs = sim.get_observations()
        filtered = observer.update(
            base_pos=raw_obs["base_position"],
            base_vel=raw_obs["base_velocity"],
            joint_states=raw_obs["joint_states"],
            contact_forces=raw_obs["contact_forces"],
        )

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
                base_pos = filtered["base_position"]
                base_orn = raw_obs["base_orientation"]
                euler = p.getEulerFromQuaternion(base_orn)
                print(f"Time: {sim_time:.1f}s, Height: {base_pos[2]:.3f} m, " +
                      f"Orient(R/P/Y): ({np.degrees(euler[0]):6.1f}°, {np.degrees(euler[1]):6.1f}°, {np.degrees(euler[2]):6.1f}°)")

    # Get final base position and orientation
    final_raw = sim.get_observations()
    final_filtered = observer.update(
        base_pos=final_raw["base_position"],
        base_vel=final_raw["base_velocity"],
        joint_states=final_raw["joint_states"],
        contact_forces=final_raw["contact_forces"],
    )
    final_pos = final_filtered["base_position"]
    final_orn = final_raw["base_orientation"]
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

    task_config = load_task_config()
    physics_params, command_limits = _build_physics_and_limits(task_config)

    # Get URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

    # Create simulation
    sim = HunterSimulation(
        urdf_path=urdf_path,
        dt=0.001,
        use_gui=use_gui,
        physics_params=physics_params,
        command_limits=command_limits,
    )
    sim.connect()
    # Start at appropriate height for standing (canonical BASE_HEIGHT)
    sim.load_robot(start_position=[0, 0, BASE_HEIGHT])
    # Use default PyBullet contact params (previous stable baseline)

    observer = Observer()
    observer.reset()

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
    target_positions = standing_config_copy()

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
    sim.reset_robot(position=[0, 0, BASE_HEIGHT], joint_positions=target_positions)

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
    initial_raw = sim.get_observations()
    initial_filtered = observer.update(
        base_pos=initial_raw["base_position"],
        base_vel=initial_raw["base_velocity"],
        joint_states=initial_raw["joint_states"],
        contact_forces=initial_raw["contact_forces"],
    )
    initial_pos = initial_filtered["base_position"]
    initial_orn = initial_raw["base_orientation"]
    initial_euler = p.getEulerFromQuaternion(initial_orn)
    print(f"Initial base height: {initial_pos[2]:.3f} m")
    print(f"Initial orientation (roll, pitch, yaw): ({np.degrees(initial_euler[0]):.1f}°, {np.degrees(initial_euler[1]):.1f}°, {np.degrees(initial_euler[2]):.1f}°)")

    while sim_time < duration:
        raw_obs = sim.get_observations()
        filtered = observer.update(
            base_pos=raw_obs["base_position"],
            base_vel=raw_obs["base_velocity"],
            joint_states=raw_obs["joint_states"],
            contact_forces=raw_obs["contact_forces"],
        )

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
                base_pos = filtered["base_position"]
                base_orn = raw_obs["base_orientation"]
                euler = p.getEulerFromQuaternion(base_orn)
                print(f"Time: {sim_time:.1f}s, Height: {base_pos[2]:.3f} m, " +
                      f"Orient(R/P/Y): ({np.degrees(euler[0]):6.1f}°, {np.degrees(euler[1]):6.1f}°, {np.degrees(euler[2]):6.1f}°)")

    # Get final base position and orientation
    final_raw = sim.get_observations()
    final_filtered = observer.update(
        base_pos=final_raw["base_position"],
        base_vel=final_raw["base_velocity"],
        joint_states=final_raw["joint_states"],
        contact_forces=final_raw["contact_forces"],
    )
    final_pos = final_filtered["base_position"]
    final_orn = final_raw["base_orientation"]
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

    Control modes (via environment variable):
    - WBC_TORQUE_CONTROL=0 (default): Position control (stable, Phase 2)
    - WBC_TORQUE_CONTROL=1: Actual torque control (Phase 3 validation)

    Args:
        duration: Test duration in seconds
        use_gui: Whether to use GUI
    """
    use_torque_control = os.environ.get("WBC_TORQUE_CONTROL", "0") != "0"
    use_hybrid_control = os.environ.get("WBC_HYBRID_CONTROL", "0") != "0"

    print("="*70)
    print("WBC STANDING TEST (MPC + Whole-Body Control)")
    print("="*70)
    if use_hybrid_control:
        print("Control mode: HYBRID (position on hips/knees, torque on ankles)")
    elif use_torque_control:
        print("Control mode: TORQUE_CONTROL (testing torque computation)")
    else:
        print("Control mode: POSITION_CONTROL (stable baseline)")
    print()

    # Load structured configs
    task_config = load_task_config()
    reference_config = load_reference_config()
    gait_config = load_gait_config()
    wbc_cfg = task_config.wbc

    # Physics and safety parameters from config
    physics_params, command_limits = _build_physics_and_limits(task_config)
    gait_default = "stance"
    if args.gait and args.gait in gait_config.gaits:
        gait_default = args.gait
    gait_schedule = GaitSchedule.from_config({"gaits": gait_config.gaits, "default": gait_default})

    # Get URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

    # Detect requested forward velocity to decide on contact solver stability
    target_velocity_cmd = None
    try:
        # Available when invoked via CLI main
        target_velocity_cmd = args.target_velocity  # type: ignore
    except Exception:
        target_velocity_cmd = None

    # Create simulation with enhanced contact solver for torque control or when commanding forward velocity
    sim = HunterSimulation(
        urdf_path=urdf_path,
        dt=0.001,
        use_gui=use_gui,
        physics_params=physics_params,
        command_limits=command_limits,
    )

    # Enable enhanced contact solver for torque/hybrid control modes
    enable_stable_contacts = (
        use_torque_control
        or use_hybrid_control
        or (target_velocity_cmd is not None and abs(target_velocity_cmd) > 1e-6)
    )
    sim.connect(enable_stable_contacts=enable_stable_contacts)

    sim.load_robot(start_position=[0, 0, BASE_HEIGHT])

    # Set contact properties for better foot-ground interaction
    sim.set_contact_properties(
        lateral_friction=physics_params.get("lateral_friction", 1.2),      # Higher friction to prevent sliding
        spinning_friction=0.2,     # Prevent foot spinning
        rolling_friction=0.05,     # Prevent foot rolling
        restitution=0.0            # No bounce
    )

    # Stable standing configuration (straight legs, feet on ground)
    standing_config = standing_config_copy()

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
    sim.reset_robot(position=[0, 0, BASE_HEIGHT], joint_positions=standing_config)

    # Initialize MPC + WBC Controller
    print("Initializing MPC + WBC Controller...")

    mpc_cfg = task_config.mpc
    mpc_params = MPCParams(
        prediction_horizon=mpc_cfg.get("horizon_steps", 20),
        control_horizon=mpc_cfg.get("control_horizon", 10),
        dt=mpc_cfg.get("dt", 0.03),  # 30Hz control default
        com_height=mpc_cfg.get("com_height", 0.55),  # Adjusted for new base height (was 0.35)
        gravity=9.81,
        Q_position=mpc_cfg.get("weights", {}).get("position", 1.0),
        Q_velocity=mpc_cfg.get("weights", {}).get("velocity", 0.1),
        R_zmp=mpc_cfg.get("weights", {}).get("zmp", 1e-6),
        max_zmp_offset=0.08
    )

    # Use tuned WBC parameters (Phase 2 + foot anchoring)
    # Enable foot anchoring for torque/hybrid control modes
    weights_cfg = wbc_cfg.get("weights", {})
    base_wbc_params = {
        "friction_coef": wbc_cfg.get("friction_coefficient", 0.6),
        "max_normal_force": wbc_cfg.get("max_normal_force", 500.0),
        "min_normal_force": wbc_cfg.get("min_normal_force", 1.0),
        "w_force_tracking": weights_cfg.get("contact_force_tracking", 1.0),
        "w_force_regularization": weights_cfg.get("joint_regularization", 0.01),
        "w_torque_regularization": weights_cfg.get("joint_regularization", 0.001),
    }

    if use_torque_control or use_hybrid_control:
        # Foot anchoring enabled - adds Cartesian stiffness to keep feet planted
        # Allow tuning via environment variables
        anchor_weight = float(os.environ.get("WBC_ANCHOR_WEIGHT", "5.0"))
        anchor_kp = float(os.environ.get("WBC_ANCHOR_KP", "100.0"))
        anchor_kd = float(os.environ.get("WBC_ANCHOR_KD", "50.0"))

        wbc_params = WBCParams(
            **base_wbc_params,
            w_foot_anchor=anchor_weight,
            foot_stiffness_kp=anchor_kp,
            foot_damping_kd=anchor_kd,
        )
        print(f"[WBC] Foot anchoring ENABLED: w={wbc_params.w_foot_anchor}, "
              f"kp={wbc_params.foot_stiffness_kp}, kd={wbc_params.foot_damping_kd}")
    else:
        # Position control mode - no need for foot anchoring
        wbc_params = WBCParams(**base_wbc_params)

    centroidal_config = CentroidalMPCConfig(
        dt=mpc_cfg.get("dt", 0.03),
        horizon_steps=mpc_cfg.get("horizon_steps", 20),
        control_horizon=mpc_cfg.get("control_horizon", 10),
        weights=mpc_cfg.get("weights", {}),
    )
    centroidal_planner = CentroidalMPC(
        centroidal_config,
        gait_schedule=gait_schedule,
        nominal_mass=12.6,  # Match PyBullet URDF mass (~12.6 kg) to align force references
    )

    controller = MPCWBCController(
        robot_id=sim.robot_id,
        joint_dict=sim.joint_dict,
        mpc_params=mpc_params,
        wbc_params=wbc_params,
        use_torque_control=use_torque_control,
        use_hybrid_control=use_hybrid_control,
        reference_targets=ReferenceTargets(
            com_height=reference_config.com_height,
            target_displacement_velocity=args.target_velocity
            if args.target_velocity is not None else reference_config.target_displacement_velocity,
            target_rotation_velocity=reference_config.target_rotation_velocity,
            default_joint_state=reference_config.default_joint_state,
        ),
        centroidal_planner=centroidal_planner,
        gait_schedule=gait_schedule,
    )

    print("Controller initialized")
    print(f"Control frequency: {1.0/mpc_params.dt:.0f}Hz")
    print(f"MPC horizon: {mpc_params.prediction_horizon} steps")

    if use_hybrid_control:
        # Hybrid mode: disable motors on torque-controlled joints (ankles), position control on others
        for joint_name in standing_config.keys():
            joint_idx = sim.get_joint_index(joint_name)
            if joint_idx is not None:
                if joint_name in controller.torque_controlled_joints:
                    # Ankle: disable for torque control
                    p.setJointMotorControl2(
                        bodyIndex=sim.robot_id,
                        jointIndex=joint_idx,
                        controlMode=p.VELOCITY_CONTROL,
                        force=0.0
                    )
                else:
                    # Hip/Knee: setup position control
                    p.setJointMotorControl2(
                        bodyIndex=sim.robot_id,
                        jointIndex=joint_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=standing_config[joint_name],
                        force=100.0,
                        maxVelocity=10.0
                    )
    elif not use_torque_control:
        # Enable position control (original stable mode)
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
    else:
        # Disable default motors for torque control
        for joint_name in standing_config.keys():
            joint_idx = sim.get_joint_index(joint_name)
            if joint_idx is not None:
                p.setJointMotorControl2(
                    bodyIndex=sim.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.VELOCITY_CONTROL,
                    force=0.0
                )

    print(f"Running simulation for {duration}s...\n")

    sim_time = 0.0
    control_dt = mpc_params.dt
    last_control = 0.0
    observer = Observer()
    observer.reset()

    while sim_time < duration:
        # Control update at MPC frequency
        if sim_time - last_control >= control_dt:
            try:
                raw_obs = sim.get_observations()
                filtered = observer.update(
                    base_pos=raw_obs["base_position"],
                    base_vel=raw_obs["base_velocity"],
                    joint_states=raw_obs["joint_states"],
                    contact_forces=raw_obs["contact_forces"],
                )
                observation = {**raw_obs, **filtered}

                # Update controller with filtered observation
                joint_commands = controller.update(control_dt, observation=observation)

                # Apply commands using unified hybrid API
                sim.apply_hybrid_command(joint_commands)

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


def run_walking_simulation(duration: float = 20.0, use_gui: bool = True, disable_estop: bool = False):
    """
    Phase 4 Position Control Walking Mode

    Uses validated position control walking system with ZMP-based CoM planning
    and full-body IK solver. Proven stable for 5+ minute continuous walking.

    Args:
        duration: Simulation duration in seconds
        use_gui: Whether to use GUI
        disable_estop: Disable emergency stop (not used in Phase 4)
    """
    print("="*70)
    print("PHASE 4 POSITION CONTROL WALKING")
    print("="*70)
    print("System: ZMP CoM Planner + Full-Body IK + Position Control")
    print("Status: Production-ready (validated 5min continuous walk)")
    print()

    task_config = load_task_config()
    physics_params, command_limits = _build_physics_and_limits(task_config)

    # Get URDF path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

    # Create simulation environment
    sim = HunterSimulation(
        urdf_path=urdf_path,
        dt=0.001,
        use_gui=use_gui,
        physics_params=physics_params,
        command_limits=command_limits,
    )
    sim.connect()
    sim.load_robot(start_position=[0, 0, BASE_HEIGHT])
    sim.set_contact_properties(
        lateral_friction=physics_params.get("lateral_friction", 1.0),
        spinning_friction=0.2,
        rolling_friction=0.05,
        restitution=0.0,
    )

    observer = Observer()
    observer.reset()

    # Set robot to standing configuration (required for IK solver initialization)
    standing_config = standing_config_copy()
    for joint_name, angle in standing_config.items():
        joint_idx = sim.get_joint_index(joint_name)
        if joint_idx is not None:
            p.resetJointState(sim.robot_id, joint_idx, angle)

    # Create filtered joint_dict (only controllable leg joints, excluding fixed joints)
    # This is required because sim.joint_dict includes fixed joints with invalid limits
    controllable_joint_dict = {}
    for joint_name in standing_config.keys():
        idx = sim.get_joint_index(joint_name)
        if idx is not None:
            controllable_joint_dict[joint_name] = idx

    # Initialize Phase 4 position control walking controller
    gait_params = GaitParams(
        step_length=0.04,      # 4cm steps (conservative, stable)
        step_height=0.02,      # 2cm lift
        step_period=2.0,       # 2s per step
        double_support_ratio=0.7,
        stance_width=0.18,
        body_height=BASE_HEIGHT,
    )

    # Reinforced ZMP/CoM planner to hold nominal height
    com_planning = SimpleCoMPlannerParams(
        com_height=BASE_HEIGHT,
        zmp_kp=26.0,   # Higher ZMP stiffness
        zmp_kd=8.0,    # More damping
        preview_time=0.5,
        dt=0.01,
        velocity_damping=0.98,
    )

    ik_params = FullBodyIKParams(
        foot_weight=200.0,        # Track feet more tightly to limit lift drift
        com_weight=80.0,          # Prioritize CoM tracking
        orientation_weight=40.0,  # Keep trunk more upright
        regularization_weight=1.0,
        com_height=BASE_HEIGHT,
        max_roll_pitch=0.087,     # Stricter upright bound (~5.0 deg)
        base_height_min=0.67,
        base_height_max=0.681,
    )

    # Create controller parameters with custom gait
    controller_params = WalkingControllerParams(
        gait=gait_params,
        com_planning=com_planning,
        ik=ik_params,
        standing_mode=False,
        enable_walking=True,
        zmp_feedback_gain=0.35,     # ZMP feedback gain (stronger to resist sag)
        zmp_correction_limit=0.10   # Max ZMP correction
    )

    print("Gait Parameters:")
    print(f"  Step length: {gait_params.step_length*100:.1f} cm")
    print(f"  Step height: {gait_params.step_height*100:.1f} cm")
    print(f"  Step period: {gait_params.step_period:.1f} s")
    print(f"  Double support ratio: {gait_params.double_support_ratio:.1f}")
    print()

    # Create controller (use filtered joint_dict to avoid fixed joints)
    controller = PositionControlWalkingController(
        robot_id=sim.robot_id,
        joint_dict=controllable_joint_dict,
        params=controller_params
    )

    # Initialize controller state
    controller.reset()

    print(f"Starting {duration}s walking simulation...")
    print("="*70)
    print()

    # Tracking variables
    start_wall_time = time.time()
    last_status_print = 0.0
    initial_pos = None

    def _foot_contact_info(sim_env: HunterSimulation):
        """Summarize per-foot contact: min contact height and summed normal force."""
        # Capture contacts where the robot is either bodyA or bodyB
        contact_points = p.getContactPoints()
        info = {
            "left": {"min_z": None, "normal": 0.0},
            "right": {"min_z": None, "normal": 0.0},
        }
        contact_links = set()
        foot_links = {
            "left": sim_env.foot_links.get("left", []),
            "right": sim_env.foot_links.get("right", []),
        }
        for cp in contact_points:
            # Determine if robot is A or B in this contact
            if cp[1] == sim_env.robot_id:
                link_idx = cp[3]
                contact_pos = cp[5] if len(cp) > 5 else None
            elif cp[2] == sim_env.robot_id:
                link_idx = cp[4]
                contact_pos = cp[6] if len(cp) > 6 else None
            else:
                continue

            contact_links.add(link_idx)
            normal_force = cp[9] if len(cp) > 9 else 0.0
            for side in ("left", "right"):
                if link_idx in foot_links.get(side, []):
                    info[side]["normal"] += normal_force
                    if contact_pos is not None:
                        z_val = contact_pos[2]
                        if info[side]["min_z"] is None or z_val < info[side]["min_z"]:
                            info[side]["min_z"] = z_val
        return info, sorted(contact_links)

    # Main control loop
    while sim.time < duration:
        raw_obs = sim.get_observations()
        filtered = observer.update(
            base_pos=raw_obs["base_position"],
            base_vel=raw_obs["base_velocity"],
            joint_states=raw_obs["joint_states"],
            contact_forces=raw_obs["contact_forces"],
        )

        # Get controller commands
        commands = controller.update(sim.dt)

        # Apply position commands
        for joint_name, target_angle in commands.items():
            joint_idx = sim.get_joint_index(joint_name)
            if joint_idx is None:
                continue

            # Use high-stiffness position control (Phase 4 approach)
            p.setJointMotorControl2(
                bodyIndex=sim.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angle,
                force=500.0,
                maxVelocity=10.0
            )

        # Step simulation
        sim.step()

        # Real-time visualization
        if use_gui:
            elapsed = time.time() - start_wall_time
            if sim.time > elapsed:
                time.sleep(sim.time - elapsed)

        # Print status every 2 seconds
        if sim.time - last_status_print >= 2.0:
            base_pos = filtered["base_position"]
            base_orn = raw_obs["base_orientation"]
            feet = raw_obs.get("foot_positions", {})
            left_foot = feet.get("left")
            right_foot = feet.get("right")
            if initial_pos is None:
                initial_pos = base_pos

            base_euler = p.getEulerFromQuaternion(base_orn)
            distance = base_pos[0] - initial_pos[0]
            phase = controller.gait_generator.phase

            contact_info, contact_links = _foot_contact_info(sim)

            def _fmt_foot(label, pos, contact):
                if pos is None:
                    return f"{label}(n/a)"
                min_z = contact["min_z"]
                normal = contact["normal"]
                min_z_str = f"{min_z:+.3f}" if min_z is not None else "n/a"
                return f"{label}(z={pos[2]:.3f},min={min_z_str},N={normal:5.1f})"

            left_tip = _fmt_foot("L", left_foot, contact_info["left"])
            right_tip = _fmt_foot("R", right_foot, contact_info["right"])

            print(f"t={sim.time:5.1f}s | Phase: {phase:4.2f} | "
                  f"Dist: {distance:5.3f}m | "
                  f"Roll: {np.degrees(base_euler[0]):+5.1f}° | "
                  f"Pitch: {np.degrees(base_euler[1]):+5.1f}° | "
                  f"H: {base_pos[2]:.3f}m | Feet: {left_tip} {right_tip} | "
                  f"contact_links={contact_links}")
            last_status_print = sim.time

    # Final metrics
    wall_time = time.time() - start_wall_time
    final_raw = sim.get_observations()
    final_filtered = observer.update(
        base_pos=final_raw["base_position"],
        base_vel=final_raw["base_velocity"],
        joint_states=final_raw["joint_states"],
        contact_forces=final_raw["contact_forces"],
    )
    final_pos = final_filtered["base_position"]
    final_orn = final_raw["base_orientation"]
    final_euler = p.getEulerFromQuaternion(final_orn)

    if initial_pos is None:
        initial_pos = final_pos

    total_distance = final_pos[0] - initial_pos[0]
    walking_speed = total_distance / duration if duration > 0 else 0.0
    expected_steps = int(duration / gait_params.step_period)

    print()
    print("="*70)
    print("WALKING SIMULATION COMPLETE")
    print("="*70)
    print(f"Duration: {sim.time:.1f}s (wall time: {wall_time:.1f}s, RT factor: {sim.time/wall_time:.2f}x)")
    print(f"Expected steps: {expected_steps} (period: {gait_params.step_period:.1f}s)")
    print(f"Forward distance: {total_distance:.3f}m")
    print(f"Walking speed: {walking_speed:.4f} m/s ({walking_speed*1000:.1f} mm/s)")
    print()
    print(f"Final position: [{final_pos[0]:+.3f}, {final_pos[1]:+.3f}, {final_pos[2]:.3f}]m")
    print(f"Final orientation: Roll={np.degrees(final_euler[0]):+.2f}°, Pitch={np.degrees(final_euler[1]):+.2f}°")
    print(f"Final height: {final_pos[2]:.3f}m (nominal: {BASE_HEIGHT:.3f}m)")
    print()

    # Assessment
    roll_deg = abs(np.degrees(final_euler[0]))
    pitch_deg = abs(np.degrees(final_euler[1]))
    height_error = abs(final_pos[2] - BASE_HEIGHT)

    success = (roll_deg < 5.0 and pitch_deg < 5.0 and height_error < 0.05)

    if success:
        print("✓ SUCCESS: Robot walked successfully")
        print(f"  - Roll stability: {roll_deg:.2f}° < 5°")
        print(f"  - Pitch stability: {pitch_deg:.2f}° < 5°")
        print(f"  - Height accuracy: {height_error*1000:.1f}mm < 50mm")
    else:
        print("⚠ COMPLETED WITH WARNINGS:")
        if roll_deg >= 5.0:
            print(f"  - Roll deviation: {roll_deg:.2f}° ≥ 5°")
        if pitch_deg >= 5.0:
            print(f"  - Pitch deviation: {pitch_deg:.2f}° ≥ 5°")
        if height_error >= 0.05:
            print(f"  - Height error: {height_error*1000:.1f}mm ≥ 50mm")

    print()
    print("Note: This uses Phase 4 position control walking (validated for 5min continuous walk).")
    print("      For robustness testing, see test_phase46_robustness.py")
    print("="*70)

    sim.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hunter bipedal robot walking simulation")
    parser.add_argument("--mode", type=str, default="standing",
                       choices=["standing", "standing-mpc", "wbc", "walking", "standing-pd-ext"],
                       help="Simulation mode")
    parser.add_argument("--duration", type=float, default=10.0,
                       help="Simulation duration (seconds)")
    parser.add_argument("--no-gui", action="store_true",
                       help="Disable GUI")
    parser.add_argument("--disable-walking-estop", action="store_true",
                       help="Disable walking-mode emergency stop (for debugging)")
    parser.add_argument("--target-velocity", type=float, default=None,
                       help="Target forward velocity for MPC/WBC planner (m/s)")
    parser.add_argument("--gait", type=str, default=None,
                       help="Gait name for centroidal planner (e.g., stance, trot)")

    args = parser.parse_args()

    use_gui = not args.no_gui

    if args.mode == "standing":
        run_standing_test(duration=args.duration, use_gui=use_gui)
    elif args.mode == "standing-mpc":
        run_standing_test_mpc(duration=args.duration, use_gui=use_gui)
    elif args.mode == "standing-pd-ext":
        task_config = load_task_config()
        physics_params, command_limits = _build_physics_and_limits(task_config)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        repo_root = os.path.abspath(os.path.join(script_dir, ".."))
        urdf_path = os.path.join(script_dir, "../models/urdf/hunter.urdf")

        import yaml

        cfg_path = os.path.join(repo_root, "config/agent_tuning.yaml")
        cfg = {}
        try:
            with open(cfg_path, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}
        except FileNotFoundError:
            cfg = {}

        runner_cfg = (cfg.get("runner") or {})
        ctrl_cfg = (cfg.get("controller") or {})
        safety_cfg = (cfg.get("safety") or {})

        base_h = runner_cfg.get("base_height", None)
        try:
            base_h = float(base_h) if base_h is not None else None
        except Exception:
            base_h = None
        if base_h is None:
            base_h = BASE_HEIGHT

        stance_name = str(ctrl_cfg.get("stance", "standing"))
        crouch_knee = ctrl_cfg.get("crouch_knee", None)
        crouch_ankle = ctrl_cfg.get("crouch_ankle", None)

        ctrl_cfg.setdefault("stance", stance_name)
        if crouch_knee is not None:
            ctrl_cfg.setdefault("crouch_knee", crouch_knee)
        if crouch_ankle is not None:
            ctrl_cfg.setdefault("crouch_ankle", crouch_ankle)

        from ext_standing_ref import stance_joint_targets

        target_positions = stance_joint_targets(
            stance_name, crouch_knee=crouch_knee, crouch_ankle=crouch_ankle
        )

        sim = HunterSimulation(
            urdf_path=urdf_path,
            dt=0.001,
            use_gui=use_gui,
            physics_params=physics_params,
            command_limits=command_limits,
        )
        sim.connect(enable_stable_contacts=True)
        sim.load_robot(start_position=[0, 0, base_h])

        # Disable default motors to avoid interference with torque control.
        for joint_name in STANDING_CONFIG.keys():
            joint_idx = sim.get_joint_index(joint_name)
            if joint_idx is not None:
                p.setJointMotorControl2(
                    bodyIndex=sim.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.VELOCITY_CONTROL,
                    force=0.0
                )

        sim.reset_robot(position=[0, 0, base_h], joint_positions=target_positions)

        from ext_pd_posture_torque import PDPostureTorque
        from ext_runner import run
        from ext_standing_ref import stance_q_ref
        from ext_pd_posture_torque import TorquePD
        from gravity_compensation import GravityCompensation

        # Seed (reproducibility)
        seed = int(runner_cfg.get("seed", 0) or 0)
        random.seed(seed)
        np.random.seed(seed)

        q_ref = stance_q_ref(stance_name, crouch_knee=crouch_knee, crouch_ankle=crouch_ankle)

        ctrl_type = str(ctrl_cfg.get("type", "torque_pd"))
        if ctrl_type == "two_stage":
            from ext_controller_two_stage import (
                TwoStagePostureController,
                PositionStageGains,
                TorqueStageGains,
            )

            warmup_seconds = float(ctrl_cfg.get("warmup_seconds", 1.0))
            blend_seconds = float(ctrl_cfg.get("blend_seconds", 0.0))
            pos_cfg = (ctrl_cfg.get("position") or {})
            tau_cfg = (ctrl_cfg.get("torque") or {})
            controller = TwoStagePostureController(
                q_ref,
                robot_id=sim.robot_id,
                warmup_seconds=warmup_seconds,
                blend_seconds=blend_seconds,
                position_gains=PositionStageGains(
                    kp=float(pos_cfg.get("kp", 0.3)),
                    kd=float(pos_cfg.get("kd", 0.1)),
                ),
                torque_gains=TorqueStageGains(
                    kp=float(tau_cfg.get("kp", 40.0)),
                    kd=float(tau_cfg.get("kd", 1.5)),
                    tau_limit=float(tau_cfg.get("tau_limit", 60.0)),
                    use_gravity_comp=bool(tau_cfg.get("use_gravity_comp", False)),
                    gravity_scale=float(tau_cfg.get("gravity_scale", 1.0)),
                    kd_blend_factor=float(tau_cfg.get("kd_blend_factor", 1.0)),
                ),
            )
        else:
            # torque_pd (backward compatible: allow flat kp/kd/tau_limit in YAML)
            gains = TorquePD(
                kp=float(ctrl_cfg.get("kp", (ctrl_cfg.get("torque") or {}).get("kp", 40.0))),
                kd=float(ctrl_cfg.get("kd", (ctrl_cfg.get("torque") or {}).get("kd", 1.5))),
                tau_limit=float(ctrl_cfg.get("tau_limit", (ctrl_cfg.get("torque") or {}).get("tau_limit", 60.0))),
            )
            controller = PDPostureTorque(
                q_ref,
                gains=gains,
                gravity_comp=GravityCompensation(sim.robot_id),
            )

        runner_cfg_effective = dict(runner_cfg)
        runner_cfg_effective["base_height"] = float(base_h)

        seconds = float(runner_cfg_effective.get("seconds", args.duration))
        control_dt = float(runner_cfg.get("control_dt", 0.01))
        settle_steps = int(runner_cfg.get("settle_steps", 0))
        log_dir = str(runner_cfg.get("log_dir", "runs"))
        if not os.path.isabs(log_dir):
            log_dir = os.path.join(repo_root, log_dir)
        run_name = str(runner_cfg.get("run_name", "standing_pd_ext"))

        result = run(
            sim,
            controller,
            seconds=seconds,
            control_dt=control_dt,
            settle_steps=settle_steps,
            log_dir=log_dir,
            run_name=run_name,
            safety_cfg=safety_cfg,
            run_meta={"seed": seed, "runner": runner_cfg_effective, "controller": ctrl_cfg, "safety": safety_cfg},
        )
        print(result)

        sim.disconnect()
    elif args.mode == "wbc":
        run_wbc_test(duration=args.duration, use_gui=use_gui)
    elif args.mode == "walking":
        run_walking_simulation(duration=args.duration, use_gui=use_gui, disable_estop=args.disable_walking_estop)

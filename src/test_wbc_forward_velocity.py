#!/usr/bin/env python3
"""
Regression test: WBC should stay upright with a small forward velocity command.
"""

import math

from main_simulation import _build_physics_and_limits
from config_loader import load_task_config, load_reference_config, load_gait_config
from simulation_env import HunterSimulation
from estimation.observer import Observer
from mpc_wbc_controller import MPCWBCController, MPCParams
from wbc_controller import WBCParams
from planning.centroidal_mpc import CentroidalMPC, CentroidalMPCConfig
from planning.gait_schedule import GaitSchedule
from planning.reference_manager import ReferenceTargets


def test_wbc_forward_velocity_stance():
    task_config = load_task_config()
    reference_config = load_reference_config()
    gait_config = load_gait_config()
    physics_params, command_limits = _build_physics_and_limits(task_config)

    sim = HunterSimulation(
        urdf_path="models/urdf/hunter.urdf",
        dt=0.001,
        use_gui=False,
        physics_params=physics_params,
        command_limits=command_limits,
    )

    # Enable stable contacts when commanding forward velocity
    sim.connect(enable_stable_contacts=True)
    sim.load_robot(start_position=[0, 0, 0.679])
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
    sim.reset_robot(position=[0, 0, 0.679], joint_positions=standing_config)
    sim.set_contact_properties(
        lateral_friction=physics_params.get("lateral_friction", 1.0),
        spinning_friction=0.2,
        rolling_friction=0.05,
        restitution=0.0,
    )

    observer = Observer()
    observer.reset()

    mpc_cfg = task_config.mpc
    mpc_params = MPCParams(
        prediction_horizon=mpc_cfg.get("horizon_steps", 20),
        control_horizon=mpc_cfg.get("control_horizon", 10),
        dt=mpc_cfg.get("dt", 0.03),
        com_height=mpc_cfg.get("com_height", 0.55),
        gravity=9.81,
        Q_position=mpc_cfg.get("weights", {}).get("position", 1.0),
        Q_velocity=mpc_cfg.get("weights", {}).get("velocity", 0.1),
        R_zmp=mpc_cfg.get("weights", {}).get("zmp", 1e-6),
        max_zmp_offset=0.08
    )

    weights_cfg = task_config.wbc.get("weights", {})
    base_wbc_params = WBCParams(
        friction_coef=task_config.wbc.get("friction_coefficient", 0.6),
        max_normal_force=task_config.wbc.get("max_normal_force", 500.0),
        min_normal_force=task_config.wbc.get("min_normal_force", 0.5),
        w_force_tracking=weights_cfg.get("contact_force_tracking", 25.0),
        w_force_regularization=weights_cfg.get("joint_regularization", 0.0001),
        w_torque_regularization=weights_cfg.get("joint_regularization", 0.001),
    )

    gait_schedule = GaitSchedule.from_config({"gaits": gait_config.gaits, "default": "stance"})
    centroidal_config = CentroidalMPCConfig(
        dt=mpc_cfg.get("dt", 0.03),
        horizon_steps=mpc_cfg.get("horizon_steps", 20),
        control_horizon=mpc_cfg.get("control_horizon", 10),
        weights=mpc_cfg.get("weights", {}),
    )
    centroidal_planner = CentroidalMPC(
        centroidal_config,
        gait_schedule=gait_schedule,
        nominal_mass=12.6,
    )

    controller = MPCWBCController(
        robot_id=sim.robot_id,
        joint_dict=sim.joint_dict,
        mpc_params=mpc_params,
        wbc_params=base_wbc_params,
        use_torque_control=False,
        use_hybrid_control=False,
        reference_targets=ReferenceTargets(
            com_height=reference_config.com_height,
            target_displacement_velocity=0.2,
            target_rotation_velocity=reference_config.target_rotation_velocity,
            default_joint_state=reference_config.default_joint_state,
        ),
        centroidal_planner=centroidal_planner,
        gait_schedule=gait_schedule,
    )

    sim_time = 0.0
    control_dt = mpc_params.dt
    last_control = 0.0
    duration = 3.0

    while sim_time < duration:
        if sim_time - last_control >= control_dt:
            raw_obs = sim.get_observations()
            filtered = observer.update(
                base_pos=raw_obs["base_position"],
                base_vel=raw_obs["base_velocity"],
                joint_states=raw_obs["joint_states"],
                contact_forces=raw_obs["contact_forces"],
            )
            observation = {**raw_obs, **filtered}
            commands = controller.update(control_dt, observation=observation)
            sim.apply_hybrid_command(commands)
            last_control = sim_time

        sim.step()
        sim_time += sim.dt

    final_raw = sim.get_observations()
    _, final_orn = final_raw["base_position"], final_raw["base_orientation"]
    roll, pitch, _ = [math.degrees(x) for x in controller._get_base_orientation()]

    sim.disconnect()

    assert abs(roll) < 10.0 and abs(pitch) < 10.0, f"WBC fell: roll={roll:.2f}, pitch={pitch:.2f}"

"""
Integrated MPC + WBC Controller for Bipedal Robot

Combines:
- MPC: High-level trajectory planning and CoM control
- WBC: Low-level force/torque optimization with constraints

This mimics the architecture used in MIT Cheetah 3 and Hunter GitHub implementation
"""

import numpy as np
import pybullet as p
from typing import Dict, Tuple, List, Any, Optional

from robot_constants import STANDING_CONFIG
from mpc_controller import LinearInvertedPendulumMPC, MPCParams
from wbc_controller import WholeBodyController, WBCParams
from wbc_tasks import (
    TaskHierarchy,
    create_body_orientation_task,
    create_body_position_task,
    create_com_tracking_task
)
from stability_metrics import compute_com, compute_com_velocity
from inverse_dynamics import InverseDynamics
from planning.centroidal_mpc import CentroidalMPC
from planning.gait_schedule import GaitSchedule
from planning.reference_manager import ReferenceTargets


class MPCWBCController:
    """
    Unified MPC + WBC Controller

    Control Flow:
    1. MPC computes optimal CoM trajectory and ZMP
    2. Tasks are created from MPC output
    3. WBC solves for ground reaction forces
    4. Forces are converted to joint torques
    """

    def __init__(self,
                 robot_id: int,
                 joint_dict: Dict[str, int],
                 mpc_params: MPCParams = None,
                 wbc_params: WBCParams = None,
                 use_torque_control: bool = False,
                 use_hybrid_control: bool = False,
                 reference_targets: Optional[ReferenceTargets] = None,
                 centroidal_planner: Optional[CentroidalMPC] = None,
                 gait_schedule: Optional[GaitSchedule] = None):
        """
        Initialize integrated controller

        Args:
            robot_id: PyBullet robot ID
            joint_dict: Dictionary mapping joint names to indices
            mpc_params: MPC parameters
            wbc_params: WBC parameters
            use_torque_control: If True, compute actual torques; if False, use position control
            use_hybrid_control: If True, use position control on hips/knees, torque on ankles
            reference_targets: Desired COM height/velocities from config
            centroidal_planner: Optional centroidal MPC planner (planning/centroidal_mpc.py)
            gait_schedule: Optional gait schedule to drive contact sequence
        """
        self.robot_id = robot_id
        self.joint_dict = joint_dict
        self.use_torque_control = use_torque_control
        self.use_hybrid_control = use_hybrid_control
        self.reference_targets = reference_targets
        self.centroidal_planner = centroidal_planner
        self.gait_schedule = gait_schedule

        # Define which joints are torque-controlled in hybrid mode (ankles only)
        self.torque_controlled_joints = ['leg_l5_joint', 'leg_r5_joint']
        self.position_controlled_joints = [
            'leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint',
            'leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint'
        ]

        # Create sub-controllers
        self.mpc = LinearInvertedPendulumMPC(mpc_params)
        self.wbc = WholeBodyController(robot_id, joint_dict, wbc_params)

        # Inverse dynamics (Phase 2.2)
        self.inv_dyn = InverseDynamics(robot_id)

        # Task hierarchy
        self.task_hierarchy = TaskHierarchy()

        # State
        self.time = 0.0

        # Foot reference positions (for anchoring)
        self.foot_reference_positions = None  # Will be set on first update

        # Torque control parameters (matching walking controller)
        # Allow override via environment variable for diagnostics
        import os
        default_torque_limit = float(os.environ.get("WBC_TORQUE_LIMIT", "20.0"))
        self.torque_limit = default_torque_limit  # Nm
        self.posture_kp = 15.0    # Joint-space posture hold
        self.posture_kd = 1.5     # Joint-space damping
        self.joint_damping_gain = 0.3  # Additional damping
        self.com_height_target = 0.65  # Target CoM height
        self.height_kp = 60.0
        self.height_kd = 6.0

        # Diagnostics
        self._diag_steps_logged = 0

        # Optional filtered observation from external observer
        self.last_observation: Optional[Dict[str, Any]] = None

        if use_torque_control or use_hybrid_control:
            if use_hybrid_control:
                print(f"[WBC] HYBRID MODE: Position control on hips/knees, torque on ankles")
                print(f"[WBC]   Torque-controlled: {self.torque_controlled_joints}")
                print(f"[WBC]   Position-controlled: {self.position_controlled_joints}")
            else:
                print(f"[WBC] FULL TORQUE MODE: All joints use torque control")
            print(f"[WBC] Torque limit: ±{self.torque_limit} Nm")
            print(f"[WBC] Posture PD: Kp={self.posture_kp}, Kd={self.posture_kd}")
            print(f"[WBC] Joint damping: {self.joint_damping_gain}")
            print(f"[WBC] Target CoM height: {self.com_height_target}m")

    def update(self, dt: float, observation: Optional[Dict[str, Any]] = None) -> Dict[str, float]:
        """
        Update controller and compute joint commands

        Args:
            dt: Time step
            observation: Optional filtered observation (base/joint/contact)

        Returns:
            joint_commands: Dictionary of joint positions/torques
        """
        self.time += dt
        self.last_observation = observation

        # 1. Get current state
        com_state = self._get_com_state()
        base_orientation = self._get_base_orientation()
        foot_positions, foot_contacts = self._get_foot_states()

        # Optional centroidal planner
        plan_forces = None
        if self.centroidal_planner:
            plan = self._run_centroidal_planner(com_state, dt)
            if plan is not None:
                # Override contact flags if available
                if plan.get("contact_schedule") and len(plan["contact_schedule"]) >= len(foot_contacts):
                    foot_contacts = plan["contact_schedule"][:len(foot_contacts)]
                plan_forces = plan.get("contact_forces")
                # Use planned COM reference for tracking
                planned_com = plan.get("com_trajectory", [])
                if planned_com:
                    target_com = planned_com[0][0:2]
                else:
                    target_com = None

                # Debug: log planner outputs occasionally
                if int(self.time * 10) % 20 == 0:  # every ~2s at 100 Hz
                    f0 = plan_forces[0][2] if plan_forces else 0.0
                    print(f"[CentroidalMPC] t={self.time:.2f}s | plan_com_z={planned_com[0][2]:.3f} | contacts={foot_contacts} | f0_z={f0:.2f}")
            else:
                target_com = None
        else:
            target_com = None

        # 2. MPC: Compute optimal CoM trajectory
        # Get support center from contact feet (fallback to CoM if no contacts)
        contact_feet = [fp for i, fp in enumerate(foot_positions) if foot_contacts[i]]
        if len(contact_feet) > 0:
            support_center = np.mean(contact_feet, axis=0)[0:2]
        else:
            # No contacts, use current CoM position
            support_center = com_state[0:2]

        # Generate reference trajectory (keeping CoM above support)
        current_com = com_state[0:2]  # x, y position
        target_com = target_com if target_com is not None else support_center  # Keep CoM above support
        current_vel = com_state[2:4]  # x_dot, y_dot

        reference_traj = self.mpc.generate_reference_trajectory(
            current_com=current_com,
            target_com=target_com,
            current_velocity=current_vel
        )

        # Compute optimal ZMP
        mpc_state = np.array([com_state[0], com_state[2], com_state[1], com_state[3]])  # [x, dx, y, dy]
        optimal_zmp, _ = self.mpc.compute_optimal_zmp(
            current_state=mpc_state,
            reference_trajectory=reference_traj,
            support_center=support_center
        )

        # 3. Create tasks from MPC output
        self.task_hierarchy.clear_tasks()

        # Task 1: Body orientation (keep upright)
        current_orn = p.getBasePositionAndOrientation(self.robot_id)[1]
        desired_orn = np.array([0, 0, 0, 1])  # Upright
        angular_vel = p.getBaseVelocity(self.robot_id)[1]

        orientation_task = create_body_orientation_task(
            current_orientation=current_orn,
            desired_orientation=desired_orn,
            current_angular_vel=np.array(angular_vel),
            kp=100.0,  # Matching GitHub kp_position=100
            kd=3.0     # Matching GitHub kd_position=3
        )
        self.task_hierarchy.add_task(orientation_task)

        # Task 2: CoM tracking (follow MPC trajectory)
        com_offset = current_com - target_com
        com_velocity = current_vel

        com_task = create_com_tracking_task(
            com_offset=com_offset,
            com_velocity=com_velocity,
            kp=50.0,
            kd=5.0
        )
        self.task_hierarchy.add_task(com_task)

        # 4. Get desired accelerations from task hierarchy
        desired_base_accel, _ = self.task_hierarchy.get_desired_acceleration()

        # Add height regulation to base acceleration
        com_z = com_state[2]
        com_dz = com_state[5]
        height_error = self.com_height_target - com_z
        desired_base_accel[2] += (
            self.height_kp * height_error
            - self.height_kd * com_dz
        )

        # Initialize foot reference positions on first update (for anchoring)
        if self.foot_reference_positions is None:
            self.foot_reference_positions = [fp.copy() for fp in foot_positions]

        # Get foot velocities for anchoring
        foot_velocities = self._get_foot_velocities()

        # 5. WBC: Compute ground reaction forces (with foot anchoring)
        ground_forces = self.wbc.compute_ground_reaction_forces(
            desired_base_accel=desired_base_accel,
            foot_positions=foot_positions,
            foot_contacts=foot_contacts,
            foot_reference_positions=self.foot_reference_positions,
            foot_velocities=foot_velocities,
            force_reference=plan_forces
        )

        if int(self.time * 10) % 20 == 0:  # every ~2s at 100 Hz
            # Log first foot planned vs solved forces for debugging
            planned_fz = plan_forces[0][2] if plan_forces else 0.0
            solved_fz = ground_forces[0][2] if len(ground_forces) > 0 else 0.0
            total_fz = float(np.sum(ground_forces[:, 2])) if hasattr(ground_forces, "__len__") else 0.0
            print(f"[WBC Forces] t={self.time:.2f}s | planned_fz={planned_fz:.2f} | solved_fz={solved_fz:.2f} | total_fz={total_fz:.2f} | contacts={foot_contacts} | mass={self.wbc.mass:.2f}")

        # 6. Convert forces to joint commands
        if self.use_hybrid_control:
            # Hybrid mode: position control on hips/knees, torque on ankles
            joint_commands = self._compute_hybrid_control(
                ground_forces=ground_forces,
                foot_contacts=foot_contacts,
                foot_positions=foot_positions
            )
        elif self.use_torque_control:
            # Use actual torque control (like walking controller)
            joint_commands = self._compute_torques_from_forces(
                ground_forces=ground_forces,
                foot_contacts=foot_contacts
            )
        else:
            # Use position control (original method)
            joint_commands = self._forces_to_joint_positions(
                ground_forces=ground_forces,
                foot_positions=foot_positions
            )

        return joint_commands

    def _get_com_state(self) -> np.ndarray:
        """
        Get CoM position and velocity

        Uses accurate computation from Phase 1 that considers all links.

        Returns:
            state: [x, y, z, dx, dy, dz]
        """
        if self.last_observation:
            base_pos = self.last_observation.get("base_position")
            base_vel = self.last_observation.get("base_velocity")
            if base_pos is not None and base_vel is not None:
                return np.array([
                    base_pos[0], base_pos[1], base_pos[2],
                    base_vel[0], base_vel[1], base_vel[2]
                ])

        # Use accurate CoM from stability_metrics (Phase 1.1) when no filtered data available
        com_pos = compute_com(self.robot_id)
        com_vel = compute_com_velocity(self.robot_id)

        return np.array([
            com_pos[0], com_pos[1], com_pos[2],
            com_vel[0], com_vel[1], com_vel[2]
        ])

    def _get_base_orientation(self) -> np.ndarray:
        """Get base orientation as Euler angles"""
        if self.last_observation and "base_orientation" in self.last_observation:
            base_orn = self.last_observation["base_orientation"]
            return np.array(p.getEulerFromQuaternion(base_orn))

        _, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        return np.array(p.getEulerFromQuaternion(base_orn))

    def _get_foot_states(self) -> Tuple[List[np.ndarray], List[bool]]:
        """
        Get foot positions and contact states

        Returns:
            foot_positions: List of foot positions
            foot_contacts: List of contact flags
        """
        if self.last_observation:
            obs_positions = self.last_observation.get("foot_positions", {})
            obs_contacts = self.last_observation.get("contacts")
            if obs_positions:
                # Ensure consistent left/right ordering
                ordered_positions = []
                ordered_contacts = []
                for side in ["left", "right"]:
                    if side in obs_positions:
                        ordered_positions.append(np.array(obs_positions[side]))
                    if obs_contacts is not None and len(obs_contacts) >= 2:
                        idx = 0 if side == "left" else 1
                        ordered_contacts.append(bool(obs_contacts[idx]))
                if ordered_positions:
                    return ordered_positions, ordered_contacts if ordered_contacts else [False] * len(ordered_positions)

        # Find foot links
        foot_links = []
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            link_name = joint_info[12].decode('utf-8')
            if 'foot' in link_name.lower() or ('l5' in link_name or 'r5' in link_name):
                foot_links.append(i)

        foot_positions = []
        foot_contacts = []

        for link_idx in foot_links:
            # Get link position
            link_state = p.getLinkState(self.robot_id, link_idx)
            foot_pos = np.array(link_state[0])
            foot_positions.append(foot_pos)

            # Check contact (simple: foot height < threshold)
            in_contact = foot_pos[2] < 0.05  # 5cm threshold
            foot_contacts.append(in_contact)

        # Ensure we have at least 2 feet
        if len(foot_positions) < 2:
            # Fallback: use leg_l5 and leg_r5 joints
            for joint_name in ['leg_l5_joint', 'leg_r5_joint']:
                if joint_name in self.joint_dict:
                    idx = self.joint_dict[joint_name]
                    link_state = p.getLinkState(self.robot_id, idx)
                    foot_pos = np.array(link_state[0])
                    foot_positions.append(foot_pos)
                    foot_contacts.append(foot_pos[2] < 0.05)

        return foot_positions, foot_contacts

    def _run_centroidal_planner(self, com_state: np.ndarray, dt: float) -> Optional[Dict[str, Any]]:
        """
        Invoke centroidal planner if provided and return its plan dict.
        """
        if self.centroidal_planner is None:
            return None

        target_vel = np.zeros(3)
        com_height = com_state[2]
        if self.reference_targets:
            target_vel[0] = self.reference_targets.target_displacement_velocity
            target_vel[2] = 0.0
            com_height = self.reference_targets.com_height

        return self.centroidal_planner.plan(
            state={
                "com_position": com_state[0:3],
                "com_velocity": com_state[3:6] if len(com_state) >= 6 else np.zeros(3),
            },
            references={
                "com_height": com_height,
                "target_velocity": target_vel,
                "contacts": [True, True],
            },
            dt=dt,
        )

    def _get_foot_velocities(self) -> List[np.ndarray]:
        """
        Get foot velocities in world frame

        Returns:
            foot_velocities: List of foot velocities [vx, vy, vz]
        """
        foot_velocities = []

        # Find foot links
        foot_links = []
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            link_name = joint_info[12].decode('utf-8')
            if 'foot' in link_name.lower() or ('l5' in link_name or 'r5' in link_name):
                foot_links.append(i)

        for link_idx in foot_links:
            # Get link velocity
            link_state = p.getLinkState(self.robot_id, link_idx, computeLinkVelocity=1)
            linear_velocity = np.array(link_state[6])  # World linear velocity
            foot_velocities.append(linear_velocity)

        # Ensure we have at least 2 feet (fallback)
        if len(foot_velocities) < 2:
            for joint_name in ['leg_l5_joint', 'leg_r5_joint']:
                if joint_name in self.joint_dict:
                    idx = self.joint_dict[joint_name]
                    link_state = p.getLinkState(self.robot_id, idx, computeLinkVelocity=1)
                    linear_velocity = np.array(link_state[6])
                    foot_velocities.append(linear_velocity)

        return foot_velocities

    def _compute_hybrid_control(self,
                                ground_forces: np.ndarray,
                                foot_contacts: List[bool],
                                foot_positions: List[np.ndarray]) -> Dict[str, float]:
        """
        Compute hybrid control: position control on hips/knees, torque on ankles

        This preserves the stable posture via position control while allowing
        WBC to adjust balance through ankle torques.

        Args:
            ground_forces: Nx3 array of ground forces from WBC QP
            foot_contacts: List of contact flags
            foot_positions: List of foot positions (for fallback)

        Returns:
            Dictionary with 'torque' and 'position' entries for each joint
        """
        # Compute full torques from WBC
        all_torques = self._compute_torques_from_forces(ground_forces, foot_contacts)

        # Get standing configuration positions
        from robot_constants import STANDING_CONFIG  # Local import to avoid cycles

        # Build hybrid commands
        hybrid_commands = {}
        for joint_name in self.joint_dict.keys():
            if joint_name in self.torque_controlled_joints:
                # Ankle: use WBC torque
                hybrid_commands[joint_name] = {
                    'mode': 'torque',
                    'value': all_torques.get(joint_name, 0.0)
                }
            elif joint_name in self.position_controlled_joints:
                # Hip/Knee: use position control
                hybrid_commands[joint_name] = {
                    'mode': 'position',
                    'value': STANDING_CONFIG.get(joint_name, 0.0)
                }
            else:
                # Unknown joint, default to zero
                hybrid_commands[joint_name] = {
                    'mode': 'position',
                    'value': 0.0
                }

        return hybrid_commands

    def _compute_torques_from_forces(self,
                                     ground_forces: np.ndarray,
                                     foot_contacts: List[bool]) -> Dict[str, float]:
        """
        Compute joint torques from ground reaction forces using actual torque control

        This mirrors the walking controller's approach:
        τ_total = τ_gravity + τ_contact + τ_posture + τ_damping

        Args:
            ground_forces: Nx3 array of ground forces from WBC QP
            foot_contacts: List of contact flags

        Returns:
            joint_torques: Dictionary of joint torques
        """
        # Get current joint states
        joint_positions, joint_velocities = self._get_actuated_joint_states()

        # 1. Gravity compensation
        gravity_torques = self.inv_dyn.compute_gravity_torques(joint_positions)

        # 2. Contact forces to joint torques via Jacobians
        foot_jacs = self._compute_contact_jacobians()
        contact_torques = np.zeros_like(gravity_torques)

        for i, foot_name in enumerate(["left_foot", "right_foot"]):
            if not foot_contacts[i]:
                continue
            J = foot_jacs.get(foot_name)
            if J is None:
                continue
            contact_torques += J.T @ ground_forces[i]

        # 3. Posture PD (hold standing configuration)
        posture_torques = self._compute_posture_pd(joint_positions, joint_velocities)

        # 4. Additional joint damping
        damping = -self.joint_damping_gain * joint_velocities

        # Total and clamp
        unclipped_torques = gravity_torques + contact_torques + posture_torques + damping
        total_torques = np.clip(unclipped_torques,
                                -self.torque_limit,
                                self.torque_limit)

        # Diagnostics (first 30 steps)
        if self._diag_steps_logged < 30:
            if self._diag_steps_logged == 0:
                print(f"[WBC Standing Torque Control] Starting diagnostics...")
                print(f"  posture_targets={np.round(self._get_posture_targets(), 3)}")
                print(f"  joint_pos={np.round(joint_positions, 3)}")

            force_norms = [float(np.linalg.norm(f)) for f in ground_forces]
            jac_ok = all(J is not None for J in foot_jacs.values())
            print(
                f"[WBC-TC] t={self.time:6.3f}s"
                f" | force_norms={np.round(force_norms, 1)}"
                f" | contact_tau={np.linalg.norm(contact_torques):6.2f}"
                f" | grav_tau={np.linalg.norm(gravity_torques):6.2f}"
                f" | posture_tau={np.linalg.norm(posture_torques):6.2f}"
                f" | damp={np.linalg.norm(damping):6.2f}"
                f" | unclipped_max={np.max(np.abs(unclipped_torques)):6.2f}"
                f" -> clipped_max={np.max(np.abs(total_torques)):6.2f}"
                f" | jac_ok={jac_ok}"
            )
            self._diag_steps_logged += 1

        return self._map_torques_to_dict(total_torques)

    def _get_actuated_joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get positions and velocities of actuated joints in order"""
        positions = []
        velocities = []

        for joint_idx in self.inv_dyn._actuated_joints:
            state = p.getJointState(self.robot_id, joint_idx)
            positions.append(state[0])
            velocities.append(state[1])

        return np.array(positions), np.array(velocities)

    def _get_posture_targets(self) -> np.ndarray:
        """Get target joint positions for standing posture (straight legs)"""
        targets = []
        for joint_idx in self.inv_dyn._actuated_joints:
            joint_name = self.inv_dyn._joint_names.get(joint_idx)
            if joint_name in STANDING_CONFIG:
                targets.append(STANDING_CONFIG[joint_name])
            else:
                targets.append(0.0)

        return np.array(targets)

    def _compute_posture_pd(self,
                            joint_positions: np.ndarray,
                            joint_velocities: np.ndarray) -> np.ndarray:
        """Compute PD torques to maintain standing posture"""
        targets = self._get_posture_targets()
        pos_err = targets - joint_positions
        return (
            self.posture_kp * pos_err
            - self.posture_kd * joint_velocities
        )

    def _compute_contact_jacobians(self) -> Dict[str, np.ndarray]:
        """
        Compute linear Jacobians for both feet (actuated joints only)

        Returns:
            Dictionary with 'left_foot' and 'right_foot' keys containing 3xN Jacobians
        """
        jacobians = {}

        # Find foot link indices
        foot_link_mapping = {
            'left_foot': None,
            'right_foot': None
        }

        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            link_name = joint_info[12].decode('utf-8')

            if 'l5' in link_name or 'left_foot' in link_name.lower():
                foot_link_mapping['left_foot'] = i
            elif 'r5' in link_name or 'right_foot' in link_name.lower():
                foot_link_mapping['right_foot'] = i

        # Get current joint states for Jacobian calculation
        joint_positions = []
        joint_velocities = []
        joint_accelerations = []

        # Base state (free-floating)
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        base_vel, base_ang_vel = p.getBaseVelocity(self.robot_id)

        # Actuated joints
        for joint_idx in self.inv_dyn._actuated_joints:
            state = p.getJointState(self.robot_id, joint_idx)
            joint_positions.append(state[0])
            joint_velocities.append(state[1])
            joint_accelerations.append(0.0)  # Assume zero acceleration for Jacobian

        # Compute Jacobians
        for foot_name, link_idx in foot_link_mapping.items():
            if link_idx is None:
                jacobians[foot_name] = None
                continue

            # Get full Jacobian (includes base DOFs)
            jac_t, jac_r = p.calculateJacobian(
                self.robot_id,
                link_idx,
                localPosition=[0, 0, 0],
                objPositions=list(joint_positions),
                objVelocities=list(joint_velocities),
                objAccelerations=list(joint_accelerations)
            )

            # Convert to numpy and extract actuated joints (skip base columns)
            jac_linear = np.array(jac_t)[:, 6:]  # Skip 6 base DOFs
            jacobians[foot_name] = jac_linear

        return jacobians

    def _map_torques_to_dict(self, torques_array: np.ndarray) -> Dict[str, float]:
        """Map torque array to dictionary with joint names"""
        torques = {name: 0.0 for name in self.joint_dict.keys()}
        for i, idx in enumerate(self.inv_dyn._actuated_joints):
            joint_name = self.inv_dyn._joint_names.get(idx)
            if joint_name in torques:
                torques[joint_name] = float(torques_array[i])
        return torques

    def _forces_to_joint_positions(self,
                                   ground_forces: np.ndarray,
                                   foot_positions: List[np.ndarray]) -> Dict[str, float]:
        """
        Convert ground reaction forces to joint position commands

        This is a simplified approach for PyBullet's position control.
        Full implementation would compute torques and use torque control.

        Args:
            ground_forces: Nx3 array of ground forces
            foot_positions: List of foot positions

        Returns:
            joint_positions: Dictionary of joint position commands
        """
        # Get current base state
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(base_orn)

        # Use PD feedback based on orientation
        pitch = euler[1]
        roll = euler[0]

        # Compute corrective joint angles (simplified)
        # Use VERY small corrections for straight-leg stability
        hip_pitch_correction = -pitch * 0.1  # Reduced from 0.5
        ankle_pitch_correction = -pitch * 0.05  # Reduced from 0.3
        hip_roll_correction = -roll * 0.1  # Reduced from 0.3

        # Base configuration: STRAIGHT LEGS (critical for stability!)
        base_config = {
            'leg_l1_joint': -0.1 + hip_roll_correction,
            'leg_l2_joint': 0.0,
            'leg_l3_joint': 0.0 + hip_pitch_correction,  # Straight, not -0.4
            'leg_l4_joint': 0.0,                         # Straight, not 0.8
            'leg_l5_joint': 0.0 - hip_pitch_correction + ankle_pitch_correction,  # Straight
            'leg_r1_joint': 0.1 - hip_roll_correction,
            'leg_r2_joint': 0.0,
            'leg_r3_joint': 0.0 + hip_pitch_correction,  # Straight, not -0.4
            'leg_r4_joint': 0.0,                         # Straight, not 0.8
            'leg_r5_joint': 0.0 - hip_pitch_correction + ankle_pitch_correction,  # Straight
        }

        return base_config


if __name__ == "__main__":
    print("MPC + WBC Integrated Controller")
    print("\nArchitecture:")
    print("  MPC → Optimal trajectory")
    print("   ↓")
    print("  Task creation")
    print("   ↓")
    print("  WBC → Ground forces (QP)")
    print("   ↓")
    print("  Joint commands")

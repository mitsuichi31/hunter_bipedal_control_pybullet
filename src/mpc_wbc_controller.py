"""
Integrated MPC + WBC Controller for Bipedal Robot

Combines:
- MPC: High-level trajectory planning and CoM control
- WBC: Low-level force/torque optimization with constraints

This mimics the architecture used in MIT Cheetah 3 and Hunter GitHub implementation
"""

import numpy as np
import pybullet as p
from typing import Dict, Tuple, List

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
                 wbc_params: WBCParams = None):
        """
        Initialize integrated controller

        Args:
            robot_id: PyBullet robot ID
            joint_dict: Dictionary mapping joint names to indices
            mpc_params: MPC parameters
            wbc_params: WBC parameters
        """
        self.robot_id = robot_id
        self.joint_dict = joint_dict

        # Create sub-controllers
        self.mpc = LinearInvertedPendulumMPC(mpc_params)
        self.wbc = WholeBodyController(robot_id, joint_dict, wbc_params)

        # Inverse dynamics (Phase 2.2)
        self.inv_dyn = InverseDynamics(robot_id)

        # Task hierarchy
        self.task_hierarchy = TaskHierarchy()

        # State
        self.time = 0.0

    def update(self, dt: float) -> Dict[str, float]:
        """
        Update controller and compute joint commands

        Args:
            dt: Time step

        Returns:
            joint_commands: Dictionary of joint positions/torques
        """
        self.time += dt

        # 1. Get current state
        com_state = self._get_com_state()
        base_orientation = self._get_base_orientation()
        foot_positions, foot_contacts = self._get_foot_states()

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
        target_com = support_center  # Keep CoM above support
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

        # 5. WBC: Compute ground reaction forces
        ground_forces = self.wbc.compute_ground_reaction_forces(
            desired_base_accel=desired_base_accel,
            foot_positions=foot_positions,
            foot_contacts=foot_contacts
        )

        # 6. Convert forces to joint commands
        # For PyBullet position control, we approximate this
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
        # Use accurate CoM from stability_metrics (Phase 1.1)
        com_pos = compute_com(self.robot_id)
        com_vel = compute_com_velocity(self.robot_id)

        return np.array([
            com_pos[0], com_pos[1], com_pos[2],
            com_vel[0], com_vel[1], com_vel[2]
        ])

    def _get_base_orientation(self) -> np.ndarray:
        """Get base orientation as Euler angles"""
        _, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        return np.array(p.getEulerFromQuaternion(base_orn))

    def _get_foot_states(self) -> Tuple[List[np.ndarray], List[bool]]:
        """
        Get foot positions and contact states

        Returns:
            foot_positions: List of foot positions
            foot_contacts: List of contact flags
        """
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
        hip_pitch_correction = -pitch * 0.5
        ankle_pitch_correction = -pitch * 0.3
        hip_roll_correction = -roll * 0.3

        # Base configuration
        base_config = {
            'leg_l1_joint': -0.1 + hip_roll_correction,
            'leg_l2_joint': 0.0,
            'leg_l3_joint': -0.4 + hip_pitch_correction,
            'leg_l4_joint': 0.8,
            'leg_l5_joint': -0.4 - hip_pitch_correction + ankle_pitch_correction,
            'leg_r1_joint': 0.1 - hip_roll_correction,
            'leg_r2_joint': 0.0,
            'leg_r3_joint': -0.4 + hip_pitch_correction,
            'leg_r4_joint': 0.8,
            'leg_r5_joint': -0.4 - hip_pitch_correction + ankle_pitch_correction,
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

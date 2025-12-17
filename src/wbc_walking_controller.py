"""
WBC-based Walking Controller for Bipedal Robot - Phase 3 Redesign

This controller integrates:
1. Contact State Machine - manages contact phases
2. Gait Generator - provides foot trajectories
3. WBC Controller - computes optimal forces/torques
4. Inverse Dynamics - maps accelerations to torques

Architecture:
Gait Generator → Contact FSM → Task Hierarchy → WBC QP → Inverse Dynamics → Torques
"""

import numpy as np
import pybullet as p
from typing import Dict, Tuple, Optional
from dataclasses import dataclass

from robot_constants import STANDING_CONFIG, standing_config_copy
from gait_generator import GaitGenerator, GaitParams
from wbc_controller import WholeBodyController, WBCParams
from wbc_tasks import (
    TaskHierarchy,
    create_body_orientation_task,
    create_com_tracking_task,
    create_swing_foot_task,
    create_stance_foot_constraint
)
from contact_state_machine import ContactStateMachine, ContactStateParams, ContactPhase
from inverse_dynamics import InverseDynamics
from stability_metrics import compute_com, compute_zmp
from inverse_kinematics import BipedalIKSolver


class ContactTransitionManager:
    """
    Manages smooth transitions during heel strike and toe off

    Gradually changes contact weights to avoid sudden force changes
    that could destabilize the robot.
    """

    def __init__(self, transition_duration: float = 0.05):
        """
        Initialize transition manager

        Args:
            transition_duration: Duration of transition (seconds), default 50ms
        """
        self.transition_duration = transition_duration
        self.in_transition = False
        self.transition_time = 0.0
        self.transition_type = None  # 'heel_strike' or 'toe_off'
        self.transition_foot = None  # 'left' or 'right'

    def start_transition(self, transition_type: str, foot_name: str):
        """
        Start a contact transition

        Args:
            transition_type: 'heel_strike' or 'toe_off'
            foot_name: 'left' or 'right'
        """
        self.in_transition = True
        self.transition_time = 0.0
        self.transition_type = transition_type
        self.transition_foot = foot_name

    def update(self, dt: float) -> float:
        """
        Update transition and get current weight

        Args:
            dt: Time step

        Returns:
            Contact weight (0 to 1)
        """
        if not self.in_transition:
            return 1.0

        self.transition_time += dt

        # Compute transition progress (0 to 1)
        progress = min(1.0, self.transition_time / self.transition_duration)

        # Compute weight based on transition type
        if self.transition_type == 'heel_strike':
            # Gradually increase weight from 0 to 1
            weight = progress
        elif self.transition_type == 'toe_off':
            # Gradually decrease weight from 1 to 0
            weight = 1.0 - progress
        else:
            weight = 1.0

        # Check if transition complete
        if progress >= 1.0:
            self.in_transition = False
            weight = 1.0 if self.transition_type == 'heel_strike' else 0.0

        return weight

    def is_transitioning(self) -> bool:
        """Check if currently in transition"""
        return self.in_transition


@dataclass
class WBCWalkingParams:
    """Parameters for WBC walking controller"""
    # Standing-only mode (freeze contacts/gait; hybrid by default)
    standing_mode: bool = False      # When True, force double support and bypass gait logic

    # Hybrid control mode (position on hips/knees, torque on ankles)
    use_hybrid_control: bool = False  # Enable hybrid control for stability

    # Control gains - Body orientation (matched to MPCWBCController)
    kp_orientation: float = 100.0   # Body orientation tracking gain (was 60.0)
    kd_orientation: float = 3.0     # Body orientation damping (was 8.0)

    # Control gains - CoM tracking (matched to MPCWBCController)
    kp_com: float = 50.0            # CoM position tracking gain (was 20.0)
    kd_com: float = 5.0             # CoM velocity damping (was 4.0)
    height_kp: float = 60.0         # Vertical height regulation gain
    height_kd: float = 6.0          # Vertical velocity damping

    # Control gains - Swing foot
    kp_swing: float = 100.0         # Swing foot position tracking
    kd_swing: float = 10.0          # Swing foot velocity damping

    # Control gains - Stance foot constraint
    kd_stance: float = 60.0         # Stance foot damping (drive velocity to zero, higher for anchoring)

    # Balance parameters
    com_height_target: float = 0.65  # Target CoM height (m, nearer to stable straight-leg)
    max_com_offset: float = 0.08     # Maximum CoM offset from center (m)

    # Safety limits
    max_roll_pitch: float = 0.35     # Maximum body tilt (rad, ~20 degrees, relaxed for tuning)
    min_com_height: float = 0.40     # Minimum acceptable CoM height (m)
    max_com_height: float = 0.70     # Maximum acceptable CoM height (m)
    max_zmp_offset: float = 0.20     # Maximum ZMP distance from support polygon center (m, relaxed for tuning)

    # Contact transitions
    transition_duration: float = 0.05  # Smooth transition duration (seconds, 50ms)

    # Control frequency
    control_dt: float = 0.001        # Control update rate (seconds, 1kHz)

    # Emergency stop
    enable_emergency_stop: bool = True   # Enable automatic emergency stop (can be disabled via CLI flag)

    # Diagnostics
    diag_freeze_contacts: bool = False    # If True, force double support and skip ZMP checks
    torque_limit: float = 20.0            # Torque clamp (Nm) for safety
    posture_kp: float = 15.0              # Joint-space posture hold (per-joint, moderate)
    posture_kd: float = 1.5               # Joint-space damping for posture hold
    diag_posture_scale: float = 0.25      # Scale posture gains when diagnostics freeze contacts
    joint_damping_gain: float = 0.3       # Additional joint velocity damping (Nm per rad/s)


class WBCWalkingController:
    """
    Walking Controller using Whole-Body Control (Phase 3)

    This controller properly handles free-floating base dynamics by using
    WBC optimization instead of IK. It coordinates:
    - Contact state management (when each foot is on ground)
    - Gait trajectory generation (where feet should be)
    - Task hierarchy (prioritized objectives)
    - Force optimization (QP solver)
    - Torque computation (inverse dynamics)
    """

    def __init__(self,
                 robot_id: int,
                 joint_dict: Dict[str, int],
                 gait_params: GaitParams = None,
                 wbc_params: WBCParams = None,
                 walking_params: WBCWalkingParams = None):
        """
        Initialize WBC walking controller

        Args:
            robot_id: PyBullet robot ID
            joint_dict: Dictionary mapping joint names to indices
            gait_params: Gait generation parameters
            wbc_params: WBC QP optimization parameters
            walking_params: Walking-specific control parameters
        """
        self.robot_id = robot_id
        self.joint_dict = joint_dict

        # Parameters
        self.gait_params = gait_params if gait_params else GaitParams()
        self.wbc_params = wbc_params if wbc_params else WBCParams()
        self.walking_params = walking_params if walking_params else WBCWalkingParams()

        # Components
        self.gait_generator = GaitGenerator(self.gait_params)

        contact_params = ContactStateParams(
            step_period=self.gait_params.step_period,
            double_support_ratio=self.gait_params.double_support_ratio,
            contact_force_threshold=5.0
        )
        self.contact_fsm = ContactStateMachine(contact_params)
        self.contact_fsm.initialize(robot_id, joint_dict)

        self.wbc = WholeBodyController(robot_id, joint_dict, self.wbc_params)
        self.inv_dyn = InverseDynamics(robot_id)
        self.task_hierarchy = TaskHierarchy()

        # Phase 3: IK solver for walking mode
        self.ik_solver = BipedalIKSolver(robot_id, joint_dict)

        # Contact transition manager
        self.transition_manager = ContactTransitionManager(
            transition_duration=self.walking_params.transition_duration
        )

        # State
        self.time = 0.0
        self.last_control_update = 0.0
        self.is_active = False
        self.previous_contact_state = (True, True)  # Track contact state for transition detection
        self._diag_steps_logged = 0  # Limit verbose diagnostic prints
        self._posture_targets = None  # Standing posture reference (actuated order)
        self._jacobian_fail_logged = False  # Avoid flooding logs
        self._foot_reference_positions = None  # Anchor points for stance feet

        # Foot link indices
        self.left_foot_link, self.right_foot_link = self._find_foot_links()

        # Hybrid control: define which joints use which control mode
        if self.walking_params.use_hybrid_control:
            self.torque_controlled_joints = ['leg_l5_joint', 'leg_r5_joint']  # Ankles
            self.position_controlled_joints = [
                'leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint',  # Left hip/knee
                'leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint',  # Right hip/knee
            ]
        else:
            self.torque_controlled_joints = list(self.joint_dict.keys())  # All joints
            self.position_controlled_joints = []

        # Standing-only path: freeze contacts/gait for stability work
        if self.walking_params.standing_mode:
            self.walking_params.diag_freeze_contacts = True
            if not self.walking_params.use_hybrid_control:
                # Hybrid split is strongly recommended when standing mode is on
                self.walking_params.use_hybrid_control = True
            print("  Standing mode enabled: contacts frozen (double support), gait disabled")

        print(f"WBC Walking Controller initialized")
        print(f"  Step period: {self.gait_params.step_period:.2f}s")
        print(f"  Step length: {self.gait_params.step_length:.3f}m")
        print(f"  Control frequency: {1.0/self.walking_params.control_dt:.0f}Hz")
        if self.walking_params.use_hybrid_control:
            print(f"  Hybrid Control: Position on hips/knees, Torque on ankles")
            print(f"    Torque-controlled: {self.torque_controlled_joints}")
            print(f"    Position-controlled: {self.position_controlled_joints}")

    def _find_foot_links(self) -> Tuple[int, int]:
        """Find link indices for feet"""
        left_foot = None
        right_foot = None

        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            link_name = joint_info[12].decode('utf-8')

            if link_name == 'leg_l5_link':
                left_foot = i
            elif link_name == 'leg_r5_link':
                right_foot = i

        if left_foot is None or right_foot is None:
            print(f"Warning: Could not find foot links. L:{left_foot}, R:{right_foot}")

        return left_foot, right_foot

    def _get_foot_state(self, foot_link_idx: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get foot position and velocity in world frame

        Args:
            foot_link_idx: Link index of the foot

        Returns:
            (position, velocity): Foot state
        """
        link_state = p.getLinkState(
            self.robot_id,
            foot_link_idx,
            computeLinkVelocity=1
        )

        position = np.array(link_state[0])  # World position
        velocity = np.array(link_state[6])  # Linear velocity

        return position, velocity

    def _ensure_foot_references(self, robot_state: Dict) -> None:
        """Capture stance foot anchor positions once for anchoring/stability tasks"""
        if self._foot_reference_positions is None:
            self._foot_reference_positions = [
                robot_state['left_foot_pos'].copy(),
                robot_state['right_foot_pos'].copy()
            ]

    def _get_robot_state(self) -> Dict:
        """Get current robot state"""
        # Base state
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        base_vel, base_ang_vel = p.getBaseVelocity(self.robot_id)

        # CoM
        com_pos = compute_com(self.robot_id)

        # Foot states
        left_pos, left_vel = self._get_foot_state(self.left_foot_link)
        right_pos, right_vel = self._get_foot_state(self.right_foot_link)

        return {
            'base_pos': np.array(base_pos),
            'base_orn': np.array(base_orn),
            'base_vel': np.array(base_vel),
            'base_ang_vel': np.array(base_ang_vel),
            'com_pos': com_pos,
            'left_foot_pos': left_pos,
            'left_foot_vel': left_vel,
            'right_foot_pos': right_pos,
            'right_foot_vel': right_vel,
        }

    def _build_task_hierarchy(self, robot_state: Dict, gait_targets: Dict,
                              current_contact: Tuple[bool, bool]) -> None:
        """
        Build task hierarchy based on current contact phase

        Args:
            robot_state: Current robot state
            gait_targets: Target positions from gait generator
            current_contact: (left, right) contact flags to build constraints
        """
        self.task_hierarchy.clear_tasks()

        left_contact, right_contact = current_contact

        # Priority 0: Stance foot constraints (highest priority)
        # OPTION B FIX: Commented out explicit stance foot constraints to avoid overconstraining QP
        # The WBC QP foot anchoring (w=10, kp=300, kd=100) handles stance stability instead
        # This matches the working MPCWBCController architecture
        # if left_contact:
        #     task = create_stance_foot_constraint(
        #         foot_name="left",
        #         foot_velocity=robot_state['left_foot_vel'],
        #         kd=self.walking_params.kd_stance
        #     )
        #     self.task_hierarchy.add_task(task)
        #
        # if right_contact:
        #     task = create_stance_foot_constraint(
        #         foot_name="right",
        #         foot_velocity=robot_state['right_foot_vel'],
        #         kd=self.walking_params.kd_stance
        #     )
        #     self.task_hierarchy.add_task(task)

        # Priority 1: Body orientation (keep upright)
        desired_orientation = np.array([0, 0, 0, 1])  # Upright (quaternion)
        task = create_body_orientation_task(
            current_orientation=robot_state['base_orn'],
            desired_orientation=desired_orientation,
            current_angular_vel=robot_state['base_ang_vel'],
            kp=self.walking_params.kp_orientation,
            kd=self.walking_params.kd_orientation
        )
        self.task_hierarchy.add_task(task)

        # Priority 1: CoM tracking (keep CoM centered over support)
        com_target_xy = np.zeros(2)  # Keep CoM centered (for now)
        com_offset_xy = robot_state['com_pos'][:2] - com_target_xy
        com_velocity_xy = robot_state['base_vel'][:2]  # Approximate

        task = create_com_tracking_task(
            com_offset=com_offset_xy,
            com_velocity=com_velocity_xy,
            kp=self.walking_params.kp_com,
            kd=self.walking_params.kd_com
        )
        self.task_hierarchy.add_task(task)

        # Priority 2: Swing foot tracking
        if not left_contact:  # Left foot swinging
            task = create_swing_foot_task(
                foot_name="left",
                current_pos=robot_state['left_foot_pos'],
                desired_pos=gait_targets['left_foot'],
                current_vel=robot_state['left_foot_vel'],
                kp=self.walking_params.kp_swing,
                kd=self.walking_params.kd_swing
            )
            self.task_hierarchy.add_task(task)

        if not right_contact:  # Right foot swinging
            task = create_swing_foot_task(
                foot_name="right",
                current_pos=robot_state['right_foot_pos'],
                desired_pos=gait_targets['right_foot'],
                current_vel=robot_state['right_foot_vel'],
                kp=self.walking_params.kp_swing,
                kd=self.walking_params.kd_swing
            )
            self.task_hierarchy.add_task(task)

    def _get_support_polygon_center(self, left_contact: bool, right_contact: bool,
                                     left_pos: np.ndarray, right_pos: np.ndarray) -> np.ndarray:
        """
        Get center of support polygon based on contact state

        Args:
            left_contact: Is left foot in contact
            right_contact: Is right foot in contact
            left_pos: Left foot position
            right_pos: Right foot position

        Returns:
            Center of support polygon [x, y]
        """
        if left_contact and right_contact:
            # Double support: center is midpoint between feet
            return (left_pos[:2] + right_pos[:2]) / 2.0
        elif left_contact:
            # Left support only
            return left_pos[:2]
        elif right_contact:
            # Right support only
            return right_pos[:2]
        else:
            # No contact (flight phase) - should not happen in normal walking
            return np.zeros(2)

    def check_stability(self, robot_state: Dict,
                        current_contact: Optional[Tuple[bool, bool]] = None) -> Tuple[bool, str]:
        """
        Enhanced stability checking with ZMP validation

        Args:
            robot_state: Current robot state
            current_contact: Optional contact flags to avoid stale FSM state

        Returns:
            (is_stable, reason): Stability flag and reason if unstable
        """
        # Check 1: Body orientation
        base_orn = robot_state['base_orn']
        euler = p.getEulerFromQuaternion(base_orn)
        roll, pitch, yaw = euler

        if abs(roll) > self.walking_params.max_roll_pitch:
            return False, f"Roll angle too large: {np.degrees(roll):.1f}°"
        if abs(pitch) > self.walking_params.max_roll_pitch:
            return False, f"Pitch angle too large: {np.degrees(pitch):.1f}°"

        # Check 2: CoM height
        com_height = robot_state['com_pos'][2]
        if com_height < self.walking_params.min_com_height:
            return False, f"CoM too low: {com_height:.3f}m"
        if com_height > self.walking_params.max_com_height:
            return False, f"CoM too high: {com_height:.3f}m"

        # Check 3: ZMP inside support polygon
        try:
            zmp = compute_zmp(self.robot_id)
            if current_contact is None:
                left_contact, right_contact = self.contact_fsm.get_contact_state()
            else:
                left_contact, right_contact = current_contact

            support_center = self._get_support_polygon_center(
                left_contact, right_contact,
                robot_state['left_foot_pos'],
                robot_state['right_foot_pos']
            )

            # Check distance from ZMP to support polygon center
            zmp_offset = np.linalg.norm(zmp[:2] - support_center)

            if zmp_offset > self.walking_params.max_zmp_offset:
                return False, f"ZMP too far from support: {zmp_offset:.3f}m"

        except Exception as e:
            # If ZMP computation fails, skip this check
            pass

        # Check 4: Foot positions reasonable (not too far from base)
        base_xy = robot_state['base_pos'][:2]
        left_xy = robot_state['left_foot_pos'][:2]
        right_xy = robot_state['right_foot_pos'][:2]

        left_dist = np.linalg.norm(left_xy - base_xy)
        right_dist = np.linalg.norm(right_xy - base_xy)

        max_foot_dist = 0.5  # Maximum reasonable foot distance (m)
        if left_dist > max_foot_dist:
            return False, f"Left foot too far from base: {left_dist:.3f}m"
        if right_dist > max_foot_dist:
            return False, f"Right foot too far from base: {right_dist:.3f}m"

        return True, "Stable"

    def _get_actuated_joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return joint positions/velocities in InverseDynamics actuated order"""
        positions = []
        velocities = []
        for idx in self.inv_dyn._actuated_joints:
            state = p.getJointState(self.robot_id, idx)
            positions.append(state[0])
            velocities.append(state[1])
        return np.array(positions), np.array(velocities)

    def _get_posture_targets(self) -> np.ndarray:
        """Standing posture targets in actuated joint order"""
        if self._posture_targets is None:
            standing_config = standing_config_copy()
            targets = []
            for idx in self.inv_dyn._actuated_joints:
                joint_name = self.inv_dyn._joint_names.get(idx)
                targets.append(standing_config.get(joint_name, 0.0))
            self._posture_targets = np.array(targets)
        return self._posture_targets

    def _compute_posture_pd(self,
                            joint_positions: np.ndarray,
                            joint_velocities: np.ndarray) -> np.ndarray:
        """Joint-space PD toward the nominal standing posture"""
        targets = self._get_posture_targets()
        pos_err = targets - joint_positions
        return (
            self.walking_params.posture_kp * pos_err
            - self.walking_params.posture_kd * joint_velocities
        )

    def _map_torques_to_dict(self, torques_array: np.ndarray) -> Dict[str, float]:
        """Map torque array (actuated order) to {joint_name: torque}"""
        torques = {name: 0.0 for name in self.joint_dict.keys()}
        for i, idx in enumerate(self.inv_dyn._actuated_joints):
            joint_name = self.inv_dyn._joint_names.get(idx)
            if joint_name in torques:
                torques[joint_name] = float(torques_array[i])
        return torques

    def _compute_contact_jacobians(self) -> Dict[str, Optional[np.ndarray]]:
        """Compute linear Jacobians for each foot, then reduce to actuated joints"""
        q_act, qd_act = self._get_actuated_joint_states()
        zeros_act = [0.0] * len(q_act)

        jacobians = {}
        for foot_name, foot_idx in (("left_foot", self.left_foot_link), ("right_foot", self.right_foot_link)):
            try:
                j_lin, j_ang = p.calculateJacobian(
                    self.robot_id,
                    foot_idx,
                    [0, 0, 0],
                    list(q_act),
                    list(qd_act),
                    zeros_act
                )
                j_lin = np.array(j_lin)
                # PyBullet prepends 6 base DoFs; keep joint columns only
                if j_lin.shape[1] > len(q_act):
                    j_lin = j_lin[:, -len(q_act):]
                jacobians[foot_name] = j_lin
            except Exception as e:
                if not self._jacobian_fail_logged:
                    print(f"Jacobian compute failed for {foot_name}: {e}")
                    print(f"  len(q_act)={len(q_act)}, len(joints)={len(self.inv_dyn._actuated_joints)}")
                    self._jacobian_fail_logged = True
                jacobians[foot_name] = None
        return jacobians

    def _compute_torques(self, robot_state: Dict, gait_targets: Dict,
                         current_contact: Tuple[bool, bool]) -> Dict[str, float]:
        """
        Compute joint torques using WBC QP + inverse dynamics pathway

        - Compute desired base acceleration from task hierarchy and height PD
        - Solve for ground reaction forces respecting contacts/friction
        - Map contact forces through foot Jacobians to joint torques
        - Add gravity compensation and mild damping
        """
        if not self.is_active:
            if self.walking_params.use_hybrid_control:
                return {name: {'mode': 'torque', 'value': 0.0} for name in self.joint_dict.keys()}
            else:
                return {name: 0.0 for name in self.joint_dict.keys()}

        # Desired base accel from tasks (6D)
        base_accel, _ = self.task_hierarchy.get_desired_acceleration()

        # Height regulation toward target CoM height
        height_error = self.walking_params.com_height_target - robot_state['com_pos'][2]
        height_vel = robot_state['base_vel'][2]
        base_accel[2] += (
            self.walking_params.height_kp * height_error
            - self.walking_params.height_kd * height_vel
        )

        # Contact state and foot positions
        left_contact, right_contact = current_contact
        foot_positions = [robot_state['left_foot_pos'], robot_state['right_foot_pos']]
        contacts = [left_contact, right_contact]
        foot_reference_positions = self._foot_reference_positions
        foot_velocities = [robot_state['left_foot_vel'], robot_state['right_foot_vel']] \
            if foot_reference_positions is not None else None

        # Solve for ground forces via WBC QP
        ground_forces = self.wbc.compute_ground_reaction_forces(
            desired_base_accel=base_accel,
            foot_positions=foot_positions,
            foot_contacts=contacts,
            foot_reference_positions=foot_reference_positions,
            foot_velocities=foot_velocities
        )

        # Foot Jacobians (linear) for actuated joints
        foot_jacs = self._compute_contact_jacobians()

        # Gravity compensation
        joint_positions, joint_velocities = self._get_actuated_joint_states()
        gravity_torques = self.inv_dyn.compute_gravity_torques(joint_positions)
        # Remove posture scaling in standing mode (Option A fix)
        if self.walking_params.standing_mode:
            posture_scale = 1.0  # Full strength for standing stability
        elif self.walking_params.diag_freeze_contacts:
            posture_scale = self.walking_params.diag_posture_scale
        else:
            posture_scale = 1.0
        posture_torques = posture_scale * self._compute_posture_pd(joint_positions, joint_velocities)

        # Map contact forces to joint torques: tau += J^T * f
        contact_torques = np.zeros_like(gravity_torques)
        for i, foot_name in enumerate(["left_foot", "right_foot"]):
            if not contacts[i]:
                continue
            J = foot_jacs.get(foot_name)
            if J is None:
                continue
            contact_torques += J.T @ ground_forces[i]

        # Additional joint damping to reduce chatter (Nm per rad/s)
        damping = -self.walking_params.joint_damping_gain * joint_velocities

        unclipped_torques = gravity_torques + contact_torques + posture_torques + damping
        total_torques = np.clip(unclipped_torques,
                                -self.walking_params.torque_limit,
                                self.walking_params.torque_limit)

        if self._diag_steps_logged < 30:
            if self._diag_steps_logged == 0:
                print(f"[WBC diag] posture_targets={np.round(self._get_posture_targets(), 3)}")
                print(f"[WBC diag] joint_pos={np.round(joint_positions, 3)}")
            force_norms = [float(np.linalg.norm(f)) for f in ground_forces]
            jac_ok = all(J is not None for J in foot_jacs.values())
            print(
                "[WBC diag]"
                f" t={self.time:6.3f}s"
                f" | base_acc={np.round(base_accel, 3)}"
                f" | height_err={height_error: .3f}"
                f" | force_norms={np.round(force_norms, 3)}"
                f" | contact_tau_norm={np.linalg.norm(contact_torques):6.3f}"
                f" | grav_tau_norm={np.linalg.norm(gravity_torques):6.3f}"
                f" | posture_norm={np.linalg.norm(posture_torques):6.3f}"
                f" | damp_norm={np.linalg.norm(damping):6.3f}"
                f" | unclipped_max={np.max(np.abs(unclipped_torques)):6.3f}"
                f" -> clipped_max={np.max(np.abs(total_torques)):6.3f}"
                f" | jacobian_ok={jac_ok}"
            )
            self._diag_steps_logged += 1

        # Return torques or hybrid commands based on mode
        if self.walking_params.use_hybrid_control:
            return self._create_hybrid_commands(total_torques)
        else:
            return self._map_torques_to_dict(total_torques)

    def _compute_position_commands(self, robot_state: Dict, gait_targets: Dict,
                                   current_contact: Tuple[bool, bool]) -> Dict[str, Dict]:
        """
        Compute joint position commands using stable position control

        Phase 2: Standing mode - straight legs with PD corrections
        Phase 3: Walking mode - IK-based swing foot tracking + stance leg stability

        Args:
            robot_state: Current robot state
            gait_targets: Target foot positions (from gait generator)
            current_contact: Contact state (left_contact, right_contact)

        Returns:
            joint_commands: Dictionary with {joint_name: {'mode': 'position', 'value': angle}}
        """
        # Get current base orientation for PD corrections
        euler = p.getEulerFromQuaternion(robot_state['base_orn'])
        roll = euler[0]
        pitch = euler[1]

        # Compute small corrective angles (matching MPCWBCController gains)
        hip_pitch_correction = -pitch * 0.1   # Pitch compensation
        ankle_pitch_correction = -pitch * 0.05
        hip_roll_correction = -roll * 0.1     # Roll compensation

        # Base configuration: straight legs (proven stable)
        standing_positions = {
            'leg_l1_joint': -0.1 + hip_roll_correction,
            'leg_l2_joint': 0.0,
            'leg_l3_joint': 0.0 + hip_pitch_correction,
            'leg_l4_joint': 0.0,
            'leg_l5_joint': 0.0 - hip_pitch_correction + ankle_pitch_correction,
            'leg_r1_joint': 0.1 - hip_roll_correction,
            'leg_r2_joint': 0.0,
            'leg_r3_joint': 0.0 + hip_pitch_correction,
            'leg_r4_joint': 0.0,
            'leg_r5_joint': 0.0 - hip_pitch_correction + ankle_pitch_correction,
        }

        # Phase 3: Walking mode with IK
        if not self.walking_params.standing_mode:
            # Walking mode: use IK for swing foot, keep stance foot stable
            left_contact, right_contact = current_contact

            # Get target foot positions from gait generator
            left_target = gait_targets.get('left_foot', None)
            right_target = gait_targets.get('right_foot', None)

            # Debug: Print foot targets and contact state
            if self.time < 3.0 and int(self.time * 1000) % 500 == 0:  # Print every 0.5s for first 3 seconds
                print(f"[IK Debug] t={self.time:.2f}s | contacts=({left_contact}, {right_contact}) | "
                      f"left_target={left_target} | right_target={right_target}")

            # Process left foot
            if left_target is not None and not left_contact:
                # Left foot is swinging - use IK
                # Convert to base frame for IK (gait generator gives world frame)
                base_pos = np.array(robot_state['base_pos'])
                left_target_base = left_target - base_pos if hasattr(left_target, '__len__') else left_target

                left_ik_solution = self.ik_solver.solve_left_leg(
                    target_position=left_target_base.tolist() if hasattr(left_target_base, 'tolist') else left_target_base
                )
                if left_ik_solution:
                    # Update left leg joint positions with IK solution
                    for joint_name, angle in left_ik_solution.items():
                        standing_positions[joint_name] = angle
                    if self.time < 3.0:
                        print(f"  [IK] Left leg IK solution: {list(left_ik_solution.values())[:3]}...")

            # Process right foot
            if right_target is not None and not right_contact:
                # Right foot is swinging - use IK
                # Convert to base frame for IK
                base_pos = np.array(robot_state['base_pos'])
                right_target_base = right_target - base_pos if hasattr(right_target, '__len__') else right_target

                right_ik_solution = self.ik_solver.solve_right_leg(
                    target_position=right_target_base.tolist() if hasattr(right_target_base, 'tolist') else right_target_base
                )
                if right_ik_solution:
                    # Update right leg joint positions with IK solution
                    for joint_name, angle in right_ik_solution.items():
                        standing_positions[joint_name] = angle
                    if self.time < 3.0:
                        print(f"  [IK] Right leg IK solution: {list(right_ik_solution.values())[:3]}...")

        # Convert to hybrid command format for main_simulation.py
        joint_commands = {}
        for joint_name, position in standing_positions.items():
            joint_commands[joint_name] = {
                'mode': 'position',
                'value': position
            }

        return joint_commands

    def _create_hybrid_commands(self, torques_array: np.ndarray) -> Dict[str, Dict]:
        """
        Create hybrid control commands (position + torque)

        Args:
            torques_array: WBC computed torques in actuated order

        Returns:
            Dictionary with {joint_name: {'mode': 'torque'|'position', 'value': float}}
        """
        # Get standing posture targets
        standing_config = STANDING_CONFIG

        # Map torques array to dict
        torques_dict = self._map_torques_to_dict(torques_array)

        # Create hybrid commands
        hybrid_commands = {}
        for joint_name in self.joint_dict.keys():
            if joint_name in self.torque_controlled_joints:
                # Torque control for ankles
                hybrid_commands[joint_name] = {
                    'mode': 'torque',
                    'value': torques_dict[joint_name]
                }
            elif joint_name in self.position_controlled_joints:
                # Position control for hips/knees
                hybrid_commands[joint_name] = {
                    'mode': 'position',
                    'value': standing_config.get(joint_name, 0.0)
                }
            else:
                # Default to zero torque for any unlisted joints
                hybrid_commands[joint_name] = {
                    'mode': 'torque',
                    'value': 0.0
                }

        return hybrid_commands

    def _detect_and_handle_transitions(self, current_contact: Tuple[bool, bool]):
        """
        Detect contact state changes and initiate smooth transitions

        Args:
            current_contact: Current contact state (left, right)
        """
        left_prev, right_prev = self.previous_contact_state
        left_curr, right_curr = current_contact

        # Detect left foot transitions
        if left_curr and not left_prev:
            # Heel strike (left foot lands)
            self.transition_manager.start_transition('heel_strike', 'left')
            print(f"t={self.time:.2f}s: Left heel strike")
        elif not left_curr and left_prev:
            # Toe off (left foot lifts)
            self.transition_manager.start_transition('toe_off', 'left')
            print(f"t={self.time:.2f}s: Left toe off")

        # Detect right foot transitions
        if right_curr and not right_prev:
            # Heel strike (right foot lands)
            self.transition_manager.start_transition('heel_strike', 'right')
            print(f"t={self.time:.2f}s: Right heel strike")
        elif not right_curr and right_prev:
            # Toe off (right foot lifts)
            self.transition_manager.start_transition('toe_off', 'right')
            print(f"t={self.time:.2f}s: Right toe off")

        # Update previous state
        self.previous_contact_state = current_contact

    def update(self, dt: float) -> Dict[str, float]:
        """
        Main control update loop with smooth transitions and safety

        Args:
            dt: Time step (seconds)

        Returns:
            Joint torques {joint_name: torque}
        """
        self.time += dt

        # Update at control frequency
        if self.time - self.last_control_update < self.walking_params.control_dt:
            # Return zero torques/commands if not time to update yet
            if self.walking_params.use_hybrid_control:
                # Return zero torque commands in hybrid format
                return {name: {'mode': 'torque', 'value': 0.0} for name in self.joint_dict.keys()}
            else:
                return {name: 0.0 for name in self.joint_dict.keys()}

        self.last_control_update = self.time

        # Get current robot state
        robot_state = self._get_robot_state()
        self._ensure_foot_references(robot_state)

        if self.walking_params.standing_mode:
            # Standing-only path: keep feet anchored at initial positions
            self.contact_fsm.phase = ContactPhase.DOUBLE_SUPPORT
            current_contact = (True, True)
            gait_targets = {
                'left_foot': self._foot_reference_positions[0],
                'right_foot': self._foot_reference_positions[1]
            }
        elif self.walking_params.diag_freeze_contacts:
            # Force double support and freeze foot targets around current pose
            current_contact = (True, True)
            gait_targets = {
                'left_foot': robot_state['left_foot_pos'],
                'right_foot': robot_state['right_foot_pos']
            }
        else:
            # Update contact state machine
            self.contact_fsm.update(dt)

            # Get current contact state
            current_contact = self.contact_fsm.get_contact_state()

            # Detect and handle contact transitions
            self._detect_and_handle_transitions(current_contact)

            # Update transition manager
            self.transition_manager.update(dt)

            # Get gait targets (foot positions)
            left_target, right_target = self.gait_generator.get_foot_trajectories(self.time)

            gait_targets = {
                'left_foot': left_target,
                'right_foot': right_target
            }

        # Enhanced safety check
        is_stable, reason = self.check_stability(robot_state, current_contact)
        if not is_stable and not self.walking_params.diag_freeze_contacts:
            if self.is_active:
                print(f"⚠ WARNING: Instability detected - {reason}")

                # Emergency stop if enabled
                if self.walking_params.enable_emergency_stop:
                    print("🛑 EMERGENCY STOP: Halting walking controller")
                    self.stop()
                    if self.walking_params.use_hybrid_control:
                        return {name: {'mode': 'torque', 'value': 0.0} for name in self.joint_dict.keys()}
                    else:
                        return {name: 0.0 for name in self.joint_dict.keys()}

        # Build task hierarchy based on contact phase
        # (Task weights will be modulated by transition_weight in future)
        self._build_task_hierarchy(robot_state, gait_targets, current_contact)

        # Phase 2 Revision: Use position control (proven stable) instead of torque control
        # Note: use_hybrid_control flag enables hybrid command format (position mode for all joints)
        if self.walking_params.use_hybrid_control:
            # Position control path (stable, matches working standing-mpc mode)
            # Returns hybrid command format: {joint_name: {'mode': 'position', 'value': angle}}
            joint_positions = self._compute_position_commands(robot_state, gait_targets, current_contact)
            return joint_positions
        else:
            # Legacy pure torque control path (empirically unstable, see BASELINE_TEST_RESULTS.md)
            # Note: This path is kept for backwards compatibility but should not be used
            torques = self._compute_torques(robot_state, gait_targets, current_contact)
            return torques

    def reset(self):
        """Reset controller state"""
        self.time = 0.0
        self.last_control_update = 0.0
        self.is_active = False
        self.previous_contact_state = (True, True)
        self.gait_generator.reset()
        self.contact_fsm.reset()
        self.task_hierarchy.clear_tasks()
        # Note: transition_manager will auto-reset when not in transition

    def start(self):
        """Activate walking controller"""
        self.is_active = True
        print("WBC Walking Controller: ACTIVE")

    def stop(self):
        """Deactivate walking controller"""
        self.is_active = False
        self.reset()
        print("WBC Walking Controller: STOPPED")

    def get_state_info(self) -> Dict:
        """Get controller state for debugging/logging"""
        return {
            'time': self.time,
            'active': self.is_active,
            'phase': self.contact_fsm.phase.name,
            'step_count': self.contact_fsm.step_count,
            'num_tasks': len(self.task_hierarchy.tasks),
            'in_transition': self.transition_manager.is_transitioning(),
            'transition_type': self.transition_manager.transition_type,
            'transition_foot': self.transition_manager.transition_foot,
        }


if __name__ == "__main__":
    print("WBC Walking Controller - Phase 3")
    print("\nThis controller integrates:")
    print("  - Contact State Machine")
    print("  - Gait Generator")
    print("  - Task Hierarchy (WBC)")
    print("  - Inverse Dynamics")
    print("\nUse via main_simulation.py --mode walking")

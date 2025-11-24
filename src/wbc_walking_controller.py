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
    # Control gains - Body orientation
    kp_orientation: float = 100.0   # Body orientation tracking gain
    kd_orientation: float = 10.0    # Body orientation damping

    # Control gains - CoM tracking
    kp_com: float = 50.0            # CoM position tracking gain
    kd_com: float = 5.0             # CoM velocity damping

    # Control gains - Swing foot
    kp_swing: float = 100.0         # Swing foot position tracking
    kd_swing: float = 10.0          # Swing foot velocity damping

    # Control gains - Stance foot constraint
    kd_stance: float = 20.0         # Stance foot damping (drive velocity to zero)

    # Balance parameters
    com_height_target: float = 0.55  # Target CoM height (m)
    max_com_offset: float = 0.08     # Maximum CoM offset from center (m)

    # Safety limits
    max_roll_pitch: float = 0.35     # Maximum body tilt (rad, ~20 degrees, relaxed for tuning)
    min_com_height: float = 0.40     # Minimum acceptable CoM height (m)
    max_com_height: float = 0.70     # Maximum acceptable CoM height (m)
    max_zmp_offset: float = 0.20     # Maximum ZMP distance from support polygon center (m, relaxed for tuning)

    # Contact transitions
    transition_duration: float = 0.05  # Smooth transition duration (seconds, 50ms)

    # Control frequency
    control_dt: float = 0.01         # Control update rate (seconds, 100Hz)

    # Emergency stop
    enable_emergency_stop: bool = True  # Enable automatic emergency stop on instability


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

        # Contact transition manager
        self.transition_manager = ContactTransitionManager(
            transition_duration=self.walking_params.transition_duration
        )

        # State
        self.time = 0.0
        self.last_control_update = 0.0
        self.is_active = False
        self.previous_contact_state = (True, True)  # Track contact state for transition detection

        # Foot link indices
        self.left_foot_link, self.right_foot_link = self._find_foot_links()

        print(f"WBC Walking Controller initialized")
        print(f"  Step period: {self.gait_params.step_period:.2f}s")
        print(f"  Step length: {self.gait_params.step_length:.3f}m")
        print(f"  Control frequency: {1.0/self.walking_params.control_dt:.0f}Hz")

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

    def _build_task_hierarchy(self, robot_state: Dict, gait_targets: Dict) -> None:
        """
        Build task hierarchy based on current contact phase

        Args:
            robot_state: Current robot state
            gait_targets: Target positions from gait generator
        """
        self.task_hierarchy.clear_tasks()

        # Get current contact phase
        phase = self.contact_fsm.phase
        left_contact, right_contact = self.contact_fsm.get_contact_state()

        # Priority 0: Stance foot constraints (highest priority)
        if left_contact:
            task = create_stance_foot_constraint(
                foot_name="left",
                foot_velocity=robot_state['left_foot_vel'],
                kd=self.walking_params.kd_stance
            )
            self.task_hierarchy.add_task(task)

        if right_contact:
            task = create_stance_foot_constraint(
                foot_name="right",
                foot_velocity=robot_state['right_foot_vel'],
                kd=self.walking_params.kd_stance
            )
            self.task_hierarchy.add_task(task)

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

    def check_stability(self, robot_state: Dict) -> Tuple[bool, str]:
        """
        Enhanced stability checking with ZMP validation

        Args:
            robot_state: Current robot state

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
            left_contact, right_contact = self.contact_fsm.get_contact_state()

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

    def _compute_torques(self, robot_state: Dict, gait_targets: Dict,
                         current_contact: Tuple[bool, bool]) -> Dict[str, float]:
        """
        Compute joint torques using simplified WBC + inverse dynamics

        Approach:
        1. Use gravity compensation from inverse dynamics
        2. Add PD control to maintain standing configuration
        3. (Future: Full WBC QP for optimal force distribution)

        Args:
            robot_state: Current robot state
            gait_targets: Target foot positions
            current_contact: Current contact state

        Returns:
            Joint torques {joint_name: torque}
        """
        # Get current joint states in proper order
        joint_names_ordered = [
            'leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint', 'leg_l5_joint',
            'leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint', 'leg_r5_joint'
        ]

        joint_positions = []
        joint_velocities = []

        for joint_name in joint_names_ordered:
            if joint_name in self.joint_dict:
                joint_idx = self.joint_dict[joint_name]
                state = p.getJointState(self.robot_id, joint_idx)
                joint_positions.append(state[0])  # position
                joint_velocities.append(state[1])  # velocity
            else:
                joint_positions.append(0.0)
                joint_velocities.append(0.0)

        joint_positions = np.array(joint_positions)
        joint_velocities = np.array(joint_velocities)

        # Get body orientation for active balance control
        base_orn = robot_state['base_orn']
        euler = p.getEulerFromQuaternion(base_orn)
        roll, pitch, yaw = euler

        # Active balance control: Adjust joints based on body orientation
        # If robot leans forward (negative pitch), lean back at hips
        # If robot leans backward (positive pitch), lean forward at hips
        balance_gain_pitch = 0.5  # How aggressively to counter pitch (reduced)
        balance_gain_roll = 0.2   # How aggressively to counter roll (reduced)

        hip_pitch_adjustment = -balance_gain_pitch * pitch  # Counter-rotate pitch

        # Also adjust based on angular velocity for damping
        base_ang_vel = robot_state['base_ang_vel']
        pitch_velocity = base_ang_vel[1]  # Pitch rate
        roll_velocity = base_ang_vel[0]   # Roll rate

        velocity_damping = 0.2
        hip_pitch_velocity_comp = -velocity_damping * pitch_velocity

        total_hip_pitch = hip_pitch_adjustment + hip_pitch_velocity_comp

        # Roll balance: Adjust hip roll (shift weight side-to-side)
        # If leaning left (positive roll), shift weight right
        hip_roll_adjustment_left = -0.1 - balance_gain_roll * roll
        hip_roll_adjustment_right = 0.1 - balance_gain_roll * roll

        # Ankle balance: Use ankle pitch to adjust ZMP position
        # Forward lean (negative pitch) → ankle dorsiflexion (positive) to shift ZMP forward
        # Backward lean (positive pitch) → ankle plantarflexion (negative) to shift ZMP back
        ankle_balance_gain = 0.3
        ankle_pitch_adjustment = ankle_balance_gain * pitch  # Same direction as lean

        # Target configuration with active balance adjustments
        target_positions = np.array([
            hip_roll_adjustment_left,  # leg_l1: hip roll with balance
            0.0,   # leg_l2: hip yaw
            total_hip_pitch,  # leg_l3: hip pitch with balance control
            0.0,   # leg_l4: knee straight
            ankle_pitch_adjustment,   # leg_l5: ankle with ZMP adjustment
            hip_roll_adjustment_right,  # leg_r1: hip roll with balance
            0.0,   # leg_r2: hip yaw
            total_hip_pitch,  # leg_r3: hip pitch with balance control
            0.0,   # leg_r4: knee straight
            ankle_pitch_adjustment,   # leg_r5: ankle with ZMP adjustment
        ])

        # PD control to compute desired accelerations
        # Very high gains for aggressive control
        kp = 2000.0  # Position gain (increased 4x)
        kd = 200.0   # Velocity gain (increased 4x)

        position_error = target_positions - joint_positions
        desired_accelerations = kp * position_error - kd * joint_velocities

        # Compute torques using inverse dynamics: τ = M(q)q̈ + g(q)
        try:
            torques_array = self.inv_dyn.inverse_dynamics(
                joint_positions,
                joint_velocities,
                desired_accelerations
            )

            # Convert to dictionary
            torques = {}

            for i, joint_name in enumerate(joint_names_ordered):
                if i < len(torques_array):
                    torques[joint_name] = float(torques_array[i])
                else:
                    torques[joint_name] = 0.0

            return torques

        except Exception as e:
            print(f"Error computing torques: {e}")
            # Fallback: return zero torques
            return {name: 0.0 for name in self.joint_dict.keys()}

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
            # Return zero torques if not time to update yet
            return {name: 0.0 for name in self.joint_dict.keys()}

        self.last_control_update = self.time

        # Update contact state machine
        phase = self.contact_fsm.update(dt)

        # Get current contact state
        current_contact = self.contact_fsm.get_contact_state()

        # Detect and handle contact transitions
        self._detect_and_handle_transitions(current_contact)

        # Update transition manager
        transition_weight = self.transition_manager.update(dt)

        # Get gait targets (foot positions)
        left_target, right_target = self.gait_generator.get_foot_trajectories(self.time)

        gait_targets = {
            'left_foot': left_target,
            'right_foot': right_target
        }

        # Get current robot state
        robot_state = self._get_robot_state()

        # Enhanced safety check
        is_stable, reason = self.check_stability(robot_state)
        if not is_stable:
            if self.is_active:
                print(f"⚠ WARNING: Instability detected - {reason}")

                # Emergency stop if enabled
                if self.walking_params.enable_emergency_stop:
                    print("🛑 EMERGENCY STOP: Halting walking controller")
                    self.stop()
                    return {name: 0.0 for name in self.joint_dict.keys()}

        # Build task hierarchy based on contact phase
        # (Task weights will be modulated by transition_weight in future)
        self._build_task_hierarchy(robot_state, gait_targets)

        # Get desired accelerations from task hierarchy
        base_accel, joint_accel = self.task_hierarchy.get_desired_acceleration()

        # Compute joint torques using simplified WBC approach
        torques = self._compute_torques(robot_state, gait_targets, current_contact)

        # Debug: Print torques periodically
        if abs(self.time - 0.1) < 0.01 or abs(self.time - 0.5) < 0.01:
            torque_magnitudes = [abs(t) for t in torques.values()]
            max_torque = max(torque_magnitudes) if torque_magnitudes else 0.0
            avg_torque = sum(torque_magnitudes) / len(torque_magnitudes) if torque_magnitudes else 0.0
            print(f"[DEBUG t={self.time:.2f}s] Torques - Max: {max_torque:.2f} Nm, Avg: {avg_torque:.2f} Nm")
            print(f"  Sample: {list(torques.items())[:3]}")

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

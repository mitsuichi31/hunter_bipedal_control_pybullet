"""
PD Controller for joint control
"""

import numpy as np
from typing import Dict, Tuple


class PDController:
    """
    Proportional-Derivative controller for joint control

    Computes torque based on position and velocity errors:
        τ = Kp * (θ_desired - θ_current) + Kd * (θ_dot_desired - θ_dot_current)
    """

    def __init__(self, kp: float = 100.0, kd: float = 10.0):
        """
        Initialize PD controller

        Args:
            kp: Proportional gain
            kd: Derivative gain
        """
        self.kp = kp
        self.kd = kd

    def compute_torque(self,
                      target_position: float,
                      current_position: float,
                      target_velocity: float = 0.0,
                      current_velocity: float = 0.0) -> float:
        """
        Compute control torque

        Args:
            target_position: Desired joint position (rad)
            current_position: Current joint position (rad)
            target_velocity: Desired joint velocity (rad/s)
            current_velocity: Current joint velocity (rad/s)

        Returns:
            Control torque
        """
        position_error = target_position - current_position
        velocity_error = target_velocity - current_velocity

        torque = self.kp * position_error + self.kd * velocity_error

        return torque

    def set_gains(self, kp: float, kd: float):
        """Update controller gains"""
        self.kp = kp
        self.kd = kd


class MultiJointPDController:
    """
    PD Controller for multiple joints

    Allows different gains for different joints
    """

    def __init__(self, default_kp: float = 100.0, default_kd: float = 10.0):
        """
        Initialize multi-joint PD controller

        Args:
            default_kp: Default proportional gain
            default_kd: Default derivative gain
        """
        self.default_kp = default_kp
        self.default_kd = default_kd

        # Joint-specific gains: joint_name -> (kp, kd)
        self.joint_gains = {}

    def set_joint_gains(self, joint_name: str, kp: float, kd: float):
        """
        Set gains for a specific joint

        Args:
            joint_name: Name of the joint
            kp: Proportional gain
            kd: Derivative gain
        """
        self.joint_gains[joint_name] = (kp, kd)

    def get_joint_gains(self, joint_name: str) -> Tuple[float, float]:
        """
        Get gains for a specific joint

        Args:
            joint_name: Name of the joint

        Returns:
            (kp, kd) tuple
        """
        return self.joint_gains.get(joint_name, (self.default_kp, self.default_kd))

    def compute_torques(self,
                       target_positions: Dict[str, float],
                       current_positions: Dict[str, float],
                       target_velocities: Dict[str, float] = None,
                       current_velocities: Dict[str, float] = None) -> Dict[str, float]:
        """
        Compute control torques for multiple joints

        Args:
            target_positions: Dictionary of target positions {joint_name: position}
            current_positions: Dictionary of current positions {joint_name: position}
            target_velocities: Dictionary of target velocities (optional)
            current_velocities: Dictionary of current velocities (optional)

        Returns:
            Dictionary of control torques {joint_name: torque}
        """
        if target_velocities is None:
            target_velocities = {name: 0.0 for name in target_positions.keys()}

        if current_velocities is None:
            current_velocities = {name: 0.0 for name in current_positions.keys()}

        torques = {}

        for joint_name, target_pos in target_positions.items():
            if joint_name not in current_positions:
                continue

            current_pos = current_positions[joint_name]
            target_vel = target_velocities.get(joint_name, 0.0)
            current_vel = current_velocities.get(joint_name, 0.0)

            # Get joint-specific gains
            kp, kd = self.get_joint_gains(joint_name)

            # Compute torque
            position_error = target_pos - current_pos
            velocity_error = target_vel - current_vel
            torque = kp * position_error + kd * velocity_error

            torques[joint_name] = torque

        return torques

    def set_default_gains(self, kp: float, kd: float):
        """Update default gains"""
        self.default_kp = kp
        self.default_kd = kd


class AdaptivePDController(MultiJointPDController):
    """
    Adaptive PD Controller with automatic gain tuning capabilities

    This controller can adjust gains based on tracking performance
    """

    def __init__(self, default_kp: float = 100.0, default_kd: float = 10.0):
        super().__init__(default_kp, default_kd)

        # Tracking history for adaptive tuning
        self.error_history = {}  # joint_name -> list of errors
        self.max_history_length = 100

    def update_error_history(self, joint_name: str, error: float):
        """Track error history for a joint"""
        if joint_name not in self.error_history:
            self.error_history[joint_name] = []

        self.error_history[joint_name].append(error)

        # Keep only recent history
        if len(self.error_history[joint_name]) > self.max_history_length:
            self.error_history[joint_name].pop(0)

    def get_tracking_performance(self, joint_name: str) -> Dict[str, float]:
        """
        Get tracking performance metrics for a joint

        Returns:
            Dictionary with 'mean_error', 'std_error', 'max_error'
        """
        if joint_name not in self.error_history or len(self.error_history[joint_name]) == 0:
            return {'mean_error': 0.0, 'std_error': 0.0, 'max_error': 0.0}

        errors = np.array(self.error_history[joint_name])
        return {
            'mean_error': np.mean(np.abs(errors)),
            'std_error': np.std(errors),
            'max_error': np.max(np.abs(errors))
        }


if __name__ == "__main__":
    # Test single joint PD controller
    print("Testing PDController:")
    pd = PDController(kp=100.0, kd=10.0)

    target_pos = 0.5
    current_pos = 0.3
    target_vel = 0.0
    current_vel = 0.1

    torque = pd.compute_torque(target_pos, current_pos, target_vel, current_vel)
    print(f"Torque: {torque:.2f} Nm")

    # Test multi-joint PD controller
    print("\nTesting MultiJointPDController:")
    multi_pd = MultiJointPDController(default_kp=100.0, default_kd=10.0)

    # Set different gains for different joints
    multi_pd.set_joint_gains("leg_l3_joint", kp=200.0, kd=20.0)
    multi_pd.set_joint_gains("leg_l4_joint", kp=150.0, kd=15.0)

    target_positions = {
        "leg_l1_joint": 0.1,
        "leg_l2_joint": 0.2,
        "leg_l3_joint": 0.5,
        "leg_l4_joint": 0.8,
    }

    current_positions = {
        "leg_l1_joint": 0.05,
        "leg_l2_joint": 0.15,
        "leg_l3_joint": 0.4,
        "leg_l4_joint": 0.7,
    }

    torques = multi_pd.compute_torques(target_positions, current_positions)

    print("\nComputed torques:")
    for joint_name, torque in torques.items():
        kp, kd = multi_pd.get_joint_gains(joint_name)
        print(f"  {joint_name}: {torque:.2f} Nm (Kp={kp}, Kd={kd})")

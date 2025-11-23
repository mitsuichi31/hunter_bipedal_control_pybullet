"""
PyBullet Simulation Environment for Hunter Bipedal Robot
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
from typing import Dict, List, Tuple, Optional


class HunterSimulation:
    """PyBullet simulation environment for Hunter bipedal robot"""

    def __init__(self,
                 urdf_path: str,
                 dt: float = 0.001,
                 use_gui: bool = True,
                 use_real_time: bool = False):
        """
        Initialize the simulation environment

        Args:
            urdf_path: Path to the Hunter URDF file
            dt: Simulation time step (seconds)
            use_gui: Whether to use GUI visualization
            use_real_time: Whether to use real-time simulation
        """
        self.urdf_path = urdf_path
        self.dt = dt
        self.use_gui = use_gui
        self.use_real_time = use_real_time

        # Physics client
        self.physics_client = None
        self.robot_id = None

        # Joint information
        self.joint_dict = {}  # name -> joint_index
        self.controllable_joints = []
        self.joint_limits = {}  # joint_index -> (lower, upper)

        # Simulation state
        self.time = 0.0

    def connect(self):
        """Connect to PyBullet physics engine"""
        if self.use_gui:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.dt)

        if self.use_real_time:
            p.setRealTimeSimulation(1)
        else:
            p.setRealTimeSimulation(0)

        print(f"Connected to PyBullet (GUI: {self.use_gui})")

    def load_robot(self, start_position: List[float] = [0, 0, 0.5],
                   start_orientation: List[float] = [0, 0, 0, 1]):
        """
        Load the Hunter robot URDF

        Args:
            start_position: Initial position [x, y, z]
            start_orientation: Initial orientation quaternion [x, y, z, w]
        """
        # Load plane
        self.plane_id = p.loadURDF("plane.urdf")

        # Load robot
        self.robot_id = p.loadURDF(
            self.urdf_path,
            basePosition=start_position,
            baseOrientation=start_orientation,
            useFixedBase=False
        )

        # Build joint information
        self._build_joint_info()

        print(f"Loaded robot from {self.urdf_path}")
        print(f"Total joints: {p.getNumJoints(self.robot_id)}")
        print(f"Controllable joints: {len(self.controllable_joints)}")

    def _build_joint_info(self):
        """Build joint information dictionaries"""
        num_joints = p.getNumJoints(self.robot_id)

        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]

            self.joint_dict[joint_name] = i

            # Only consider revolute joints (type 0)
            if joint_type == p.JOINT_REVOLUTE:
                self.controllable_joints.append(i)
                lower_limit = joint_info[8]
                upper_limit = joint_info[9]
                self.joint_limits[i] = (lower_limit, upper_limit)

        # Print joint information
        print("\nControllable joints:")
        for idx in self.controllable_joints:
            name = self.get_joint_name(idx)
            limits = self.joint_limits[idx]
            print(f"  {name} (index {idx}): limits [{limits[0]:.2f}, {limits[1]:.2f}] rad")

    def get_joint_name(self, joint_index: int) -> str:
        """Get joint name from index"""
        for name, idx in self.joint_dict.items():
            if idx == joint_index:
                return name
        return f"joint_{joint_index}"

    def get_joint_index(self, joint_name: str) -> Optional[int]:
        """Get joint index from name"""
        return self.joint_dict.get(joint_name, None)

    def get_joint_states(self) -> Dict[str, Tuple[float, float, float, float]]:
        """
        Get current joint states

        Returns:
            Dictionary mapping joint name to (position, velocity, reaction_forces, applied_torque)
        """
        joint_states = p.getJointStates(self.robot_id, self.controllable_joints)

        result = {}
        for i, idx in enumerate(self.controllable_joints):
            name = self.get_joint_name(idx)
            result[name] = joint_states[i]

        return result

    def get_base_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Get base link state

        Returns:
            position, orientation (quaternion), linear_velocity, angular_velocity
        """
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        lin_vel, ang_vel = p.getBaseVelocity(self.robot_id)

        return (np.array(pos), np.array(orn),
                np.array(lin_vel), np.array(ang_vel))

    def set_joint_positions(self, joint_positions: Dict[str, float]):
        """
        Set joint positions using position control

        Args:
            joint_positions: Dictionary mapping joint name to target position
        """
        for joint_name, position in joint_positions.items():
            joint_idx = self.get_joint_index(joint_name)
            if joint_idx is not None:
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=position
                )

    def set_joint_torques(self, joint_torques: Dict[str, float]):
        """
        Set joint torques using torque control

        Args:
            joint_torques: Dictionary mapping joint name to target torque
        """
        for joint_name, torque in joint_torques.items():
            joint_idx = self.get_joint_index(joint_name)
            if joint_idx is not None:
                # Disable default motor to allow torque control
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.VELOCITY_CONTROL,
                    force=0
                )

                # Apply torque
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.TORQUE_CONTROL,
                    force=torque
                )

    def reset_robot(self,
                   position: List[float] = [0, 0, 0.5],
                   orientation: List[float] = [0, 0, 0, 1],
                   joint_positions: Optional[Dict[str, float]] = None):
        """
        Reset robot to initial state

        Args:
            position: Base position [x, y, z]
            orientation: Base orientation quaternion [x, y, z, w]
            joint_positions: Initial joint positions (optional)
        """
        # Reset base
        p.resetBasePositionAndOrientation(self.robot_id, position, orientation)
        p.resetBaseVelocity(self.robot_id, [0, 0, 0], [0, 0, 0])

        # Reset joints
        if joint_positions is None:
            # Set all joints to zero
            for joint_idx in self.controllable_joints:
                p.resetJointState(self.robot_id, joint_idx, 0.0, 0.0)
        else:
            for joint_name, pos in joint_positions.items():
                joint_idx = self.get_joint_index(joint_name)
                if joint_idx is not None:
                    p.resetJointState(self.robot_id, joint_idx, pos, 0.0)

        self.time = 0.0
        print("Robot reset")

    def step(self):
        """Step the simulation forward by one time step"""
        if not self.use_real_time:
            p.stepSimulation()
        self.time += self.dt

    def get_end_effector_position(self, link_name: str) -> np.ndarray:
        """
        Get the world position of an end effector link

        Args:
            link_name: Name of the link

        Returns:
            3D position as numpy array
        """
        link_idx = self.joint_dict.get(link_name, None)
        if link_idx is None:
            # Try to find it as a link
            for i in range(p.getNumJoints(self.robot_id)):
                info = p.getJointInfo(self.robot_id, i)
                if info[12].decode('utf-8') == link_name:
                    link_idx = i
                    break

        if link_idx is not None:
            link_state = p.getLinkState(self.robot_id, link_idx)
            return np.array(link_state[0])  # World position
        else:
            print(f"Warning: Link '{link_name}' not found")
            return np.array([0, 0, 0])

    def get_contact_points(self) -> List:
        """Get contact points on the robot"""
        return p.getContactPoints(bodyA=self.robot_id)

    def disconnect(self):
        """Disconnect from PyBullet"""
        if self.physics_client is not None:
            p.disconnect()
            print("Disconnected from PyBullet")

    def __del__(self):
        """Cleanup on deletion"""
        self.disconnect()


if __name__ == "__main__":
    # Simple test
    import os

    urdf_path = os.path.join(os.path.dirname(__file__),
                             "../models/urdf/hunter.urdf")

    sim = HunterSimulation(urdf_path=urdf_path, use_gui=True)
    sim.connect()
    sim.load_robot()

    print("\nRunning test simulation...")
    for i in range(1000):
        sim.step()
        time.sleep(sim.dt)

    sim.disconnect()

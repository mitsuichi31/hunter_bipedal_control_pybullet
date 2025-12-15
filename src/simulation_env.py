"""
PyBullet Simulation Environment for Hunter Bipedal Robot
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
from typing import Dict, List, Tuple, Optional, Any


class HunterSimulation:
    """PyBullet simulation environment for Hunter bipedal robot"""

    def __init__(self,
                 urdf_path: str,
                 dt: float = 0.001,
                 use_gui: bool = True,
                 use_real_time: bool = False,
                 physics_params: Optional[Dict[str, Any]] = None,
                 command_limits: Optional[Dict[str, float]] = None):
        """
        Initialize the simulation environment

        Args:
            urdf_path: Path to the Hunter URDF file
            dt: Simulation time step (seconds)
            use_gui: Whether to use GUI visualization
            use_real_time: Whether to use real-time simulation
            physics_params: Optional physics configuration (friction, ERP/CFM, solver iters)
            command_limits: Optional safety limits for torque/velocity clamping
        """
        self.urdf_path = urdf_path
        self.dt = dt
        self.use_gui = use_gui
        self.use_real_time = use_real_time
        self.physics_params = physics_params or {}
        self.command_limits = command_limits or {}

        # Physics client
        self.physics_client = None
        self.robot_id = None

        # Joint information
        self.joint_dict = {}  # name -> joint_index
        self.controllable_joints = []
        self.joint_limits = {}  # joint_index -> (lower, upper)

        # Foot links (populated after load)
        self.foot_links: Dict[str, List[int]] = {"left": [], "right": []}

        # Simulation state
        self.time = 0.0

    def connect(self, enable_stable_contacts: bool = False):
        """
        Connect to PyBullet physics engine

        Args:
            enable_stable_contacts: If True, use enhanced contact solver settings for torque control stability
        """
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

        # Apply optional physics engine parameters
        self._apply_physics_parameters()

        # Enhanced contact solver settings for torque control stability
        if enable_stable_contacts:
            print("Enabling enhanced contact solver settings for torque control...")

            # Increase solver iterations for better constraint accuracy
            # Default is 50, increase to 150-300 for torque control
            p.setPhysicsEngineParameter(numSolverIterations=200)

            # Add substeps for smoother contact resolution
            # Effectively increases simulation rate without changing control timestep
            p.setPhysicsEngineParameter(numSubSteps=4)

            # Prevent premature contact breaking
            # Default is 0.02m, reduce to keep contacts more stable
            p.setPhysicsEngineParameter(contactBreakingThreshold=0.001)

            # ERP (Error Reduction Parameter) - how aggressively to fix constraint violations
            # Default ~0.2, lower values are more stable but less accurate
            p.setPhysicsEngineParameter(erp=0.1)

            # Contact-specific ERP - separate from joint constraints
            # Lower for softer, more stable contacts
            p.setPhysicsEngineParameter(contactERP=0.05)

            # Enable friction anchors - helps prevent foot sliding
            p.setPhysicsEngineParameter(enableConeFriction=1)

            print("  numSolverIterations: 200 (default: 50)")
            print("  numSubSteps: 4 (default: 1)")
            print("  contactBreakingThreshold: 0.001m (default: 0.02m)")
            print("  erp: 0.1 (default: ~0.2)")
            print("  contactERP: 0.05")
            print("  enableConeFriction: 1")

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
        self._identify_feet()

        print(f"Loaded robot from {self.urdf_path}")
        print(f"Total joints: {p.getNumJoints(self.robot_id)}")
        print(f"Controllable joints: {len(self.controllable_joints)}")

    def set_contact_properties(self, lateral_friction: float = 1.0,
                               spinning_friction: float = 0.1,
                               rolling_friction: float = 0.01,
                               restitution: float = 0.0):
        """
        Set contact properties for robot feet

        Args:
            lateral_friction: Friction coefficient for sliding (default 1.0)
            spinning_friction: Friction for spinning motion (default 0.1)
            rolling_friction: Friction for rolling motion (default 0.01)
            restitution: Bounciness, 0=no bounce, 1=perfect bounce (default 0.0)
        """
        # Find foot links (leg_l5 and leg_r5 links)
        foot_links = self.foot_links.get("left", []) + self.foot_links.get("right", [])
        if not foot_links:
            num_joints = p.getNumJoints(self.robot_id)
            for i in range(num_joints):
                joint_info = p.getJointInfo(self.robot_id, i)
                link_name = joint_info[12].decode('utf-8')
                if 'foot' in link_name.lower() or 'l5' in link_name or 'r5' in link_name:
                    foot_links.append(i)

        if len(foot_links) == 0:
            print("Warning: No foot links found for contact property setting")
            return

        print(f"Setting contact properties for {len(foot_links)} foot links:")
        print(f"  lateral_friction={lateral_friction}")
        print(f"  spinning_friction={spinning_friction}")
        print(f"  rolling_friction={rolling_friction}")
        print(f"  restitution={restitution}")

        for link_idx in foot_links:
            # Set contact properties for this link
            p.changeDynamics(
                self.robot_id,
                link_idx,
                lateralFriction=lateral_friction,
                spinningFriction=spinning_friction,
                rollingFriction=rolling_friction,
                restitution=restitution,
                contactStiffness=1e4,  # High stiffness for rigid contact
                contactDamping=1e3     # High damping for stable contact
            )

            joint_info = p.getJointInfo(self.robot_id, link_idx)
            link_name = joint_info[12].decode('utf-8')
            print(f"    Link {link_idx} ({link_name}): properties set")

        # Also set ground plane properties
        p.changeDynamics(
            self.plane_id,
            -1,  # Base link
            lateralFriction=lateral_friction,
            spinningFriction=spinning_friction,
            rollingFriction=rolling_friction,
            restitution=restitution
        )
        print(f"    Ground plane: properties set")

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

    def get_contact_forces(self) -> Dict[str, np.ndarray]:
        """
        Aggregate contact forces per foot link using PyBullet contact data.

        Returns:
            Mapping 'left'/'right' -> force vector (world frame)
        """
        forces = {"left": np.zeros(3), "right": np.zeros(3)}
        contact_points = p.getContactPoints(bodyA=self.robot_id)
        for cp in contact_points:
            link_idx = cp[3]  # linkIndexA
            normal_force = cp[9]
            normal = np.array(cp[7])  # contact normal on B towards A
            force_vec = normal * normal_force

            if link_idx in self.foot_links.get("left", []):
                forces["left"] += force_vec
            elif link_idx in self.foot_links.get("right", []):
                forces["right"] += force_vec
        return forces

    def get_foot_positions(self) -> Dict[str, np.ndarray]:
        """
        Get world-frame positions of the feet (left/right).
        """
        positions = {}
        for side, links in self.foot_links.items():
            if not links:
                continue
            # Use first matching link
            link_idx = links[0]
            link_state = p.getLinkState(self.robot_id, link_idx)
            positions[side] = np.array(link_state[0])
        return positions

    def get_observations(self) -> Dict[str, Any]:
        """
        Fetch raw observations from the simulator.

        Returns:
            Dict with base pose/vel, joint states, contact forces, and foot positions.
        """
        base_pos, base_orn, lin_vel, ang_vel = self.get_base_state()
        joint_states_raw = self.get_joint_states()
        joint_states = {name: (state[0], state[1]) for name, state in joint_states_raw.items()}

        return {
            "time": self.time,
            "base_position": base_pos,
            "base_orientation": base_orn,
            "base_velocity": lin_vel,
            "base_angular_velocity": ang_vel,
            "joint_states": joint_states,
            "contact_forces": self.get_contact_forces(),
            "foot_positions": self.get_foot_positions(),
        }

    def apply_hybrid_command(self, commands: Dict[str, Any]):
        """
        Apply hybrid joint commands (position/velocity/gains + torque feedforward).

        Args:
            commands: Mapping from joint name to command. Supported forms:
              - float/int: interpreted as position command
              - {'mode': 'position', 'value': target, 'kp': ..., 'kd': ..., 'velocity': ...}
              - {'mode': 'torque', 'value': torque}
              - {'mode': 'hybrid', 'position': ..., 'velocity': ..., 'kp': ..., 'kd': ..., 'torque': ...}
        """
        if commands is None:
            return

        max_torque = self.command_limits.get("max_torque")
        max_velocity = self.command_limits.get("max_velocity")

        # Snapshot joint states for PD/hybrid computations
        joint_states = self.get_joint_states()

        for joint_name, cmd in commands.items():
            joint_idx = self.get_joint_index(joint_name)
            if joint_idx is None:
                continue

            # Normalize command structure
            if isinstance(cmd, (int, float)):
                mode = "position"
                target_pos = float(cmd)
                target_vel = 0.0
                kp = None
                kd = None
                ff_torque = 0.0
            elif isinstance(cmd, dict):
                mode = cmd.get("mode", "position")
                ff_torque = float(cmd.get("torque", cmd.get("value", 0.0)))
                target_pos = float(cmd.get("position", cmd.get("value", 0.0)))
                target_vel = float(cmd.get("velocity", 0.0))
                kp = cmd.get("kp", None)
                kd = cmd.get("kd", None)
            else:
                continue

            if mode == "torque":
                torque = self._clamp(ff_torque, max_torque)
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.VELOCITY_CONTROL,
                    force=0.0
                )
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.TORQUE_CONTROL,
                    force=torque
                )
                continue

            if mode == "hybrid":
                # Compute PD + feedforward torque
                state = joint_states.get(joint_name, (0.0, 0.0, None, 0.0))
                pos = state[0]
                vel = state[1]
                kp_val = float(kp) if kp is not None else 0.0
                kd_val = float(kd) if kd is not None else 0.0
                torque_cmd = kp_val * (target_pos - pos) + kd_val * (target_vel - vel) + ff_torque
                torque = self._clamp(torque_cmd, max_torque)
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.VELOCITY_CONTROL,
                    force=0.0
                )
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.TORQUE_CONTROL,
                    force=torque
                )
                continue

            # Position mode (default)
            kwargs = {
                "bodyIndex": self.robot_id,
                "jointIndex": joint_idx,
                "controlMode": p.POSITION_CONTROL,
                "targetPosition": target_pos,
            }
            if kp is not None:
                kwargs["positionGain"] = float(kp)
            if kd is not None:
                kwargs["velocityGain"] = float(kd)
            if max_velocity is not None:
                kwargs["maxVelocity"] = max_velocity
            if max_torque is not None:
                kwargs["force"] = max_torque
            p.setJointMotorControl2(**kwargs)

    def apply_external_force(self, force: List[float], position: Optional[List[float]] = None,
                            link_index: int = -1, flags: int = p.WORLD_FRAME):
        """
        Apply external force to the robot (for disturbance/push testing)

        Args:
            force: Force vector [fx, fy, fz] in Newtons
            position: Position to apply force (default: center of mass of link)
            link_index: Link index to apply force to (-1 for base link)
            flags: Reference frame (p.WORLD_FRAME or p.LINK_FRAME)
        """
        if position is None:
            # Apply at center of mass of the link
            p.applyExternalForce(
                objectUniqueId=self.robot_id,
                linkIndex=link_index,
                forceObj=force,
                posObj=[0, 0, 0],  # At COM
                flags=p.LINK_FRAME
            )
        else:
            p.applyExternalForce(
                objectUniqueId=self.robot_id,
                linkIndex=link_index,
                forceObj=force,
                posObj=position,
                flags=flags
            )

    def apply_external_torque(self, torque: List[float], link_index: int = -1,
                             flags: int = p.WORLD_FRAME):
        """
        Apply external torque to the robot

        Args:
            torque: Torque vector [tx, ty, tz] in N·m
            link_index: Link index to apply torque to (-1 for base link)
            flags: Reference frame (p.WORLD_FRAME or p.LINK_FRAME)
        """
        p.applyExternalTorque(
            objectUniqueId=self.robot_id,
            linkIndex=link_index,
            torqueObj=torque,
            flags=flags
        )

    def disconnect(self):
        """Disconnect from PyBullet"""
        if self.physics_client is not None and p.isConnected(self.physics_client):
            p.disconnect(self.physics_client)
            print("Disconnected from PyBullet")
        self.physics_client = None

    def __del__(self):
        """Cleanup on deletion"""
        self.disconnect()

    def _apply_physics_parameters(self):
        """
        Apply optional physics parameters (ERP/CFM, solver iterations) from config.
        """
        if not self.physics_params:
            return

        params = {}
        if "num_solver_iterations" in self.physics_params and self.physics_params["num_solver_iterations"] is not None:
            params["numSolverIterations"] = self.physics_params["num_solver_iterations"]
        if "num_sub_steps" in self.physics_params and self.physics_params["num_sub_steps"] is not None:
            params["numSubSteps"] = self.physics_params["num_sub_steps"]
        if "erp" in self.physics_params and self.physics_params["erp"] is not None:
            params["erp"] = self.physics_params["erp"]
        if "contact_erp" in self.physics_params and self.physics_params["contact_erp"] is not None:
            params["contactERP"] = self.physics_params["contact_erp"]
        if "contact_cfm" in self.physics_params and self.physics_params["contact_cfm"] is not None:
            params["contactCFM"] = self.physics_params["contact_cfm"]

        if params:
            p.setPhysicsEngineParameter(**params)

    def _identify_feet(self):
        """
        Populate left/right foot link indices based on link names.
        """
        self.foot_links = {"left": [], "right": []}
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            link_name = joint_info[12].decode('utf-8').lower()
            if "left" in link_name or "l5" in link_name:
                self.foot_links["left"].append(i)
            elif "right" in link_name or "r5" in link_name:
                self.foot_links["right"].append(i)

    @staticmethod
    def _clamp(value: float, limit: Optional[float]) -> float:
        if limit is None or limit <= 0:
            return value
        return max(min(value, limit), -limit)


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

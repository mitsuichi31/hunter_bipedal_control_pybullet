"""
Configuration loader for simulation parameters
"""

import yaml
import os
from typing import Dict, Any
from gait_generator import GaitParams


class SimulationConfig:
    """Configuration container for simulation parameters"""

    def __init__(self, config_dict: Dict[str, Any]):
        """
        Initialize from configuration dictionary

        Args:
            config_dict: Configuration dictionary loaded from YAML
        """
        self.config = config_dict

        # Parse simulation settings
        sim_config = config_dict.get('simulation', {})
        self.dt = sim_config.get('dt', 0.001)
        self.use_gui = sim_config.get('use_gui', True)
        self.use_real_time = sim_config.get('use_real_time', False)
        self.duration = sim_config.get('duration', 20.0)

        # Parse initial state
        init_state = config_dict.get('initial_state', {})
        self.initial_position = init_state.get('position', [0.0, 0.0, 0.5])
        self.initial_orientation = init_state.get('orientation', [0.0, 0.0, 0.0, 1.0])

        # Parse gait parameters
        gait_config = config_dict.get('gait', {})
        self.gait_params = GaitParams(
            step_length=gait_config.get('step_length', 0.08),
            step_height=gait_config.get('step_height', 0.04),
            step_period=gait_config.get('step_period', 1.2),
            stance_width=gait_config.get('stance_width', 0.18),
            body_height=gait_config.get('body_height', 0.45),
            double_support_ratio=gait_config.get('double_support_ratio', 0.2)
        )

        # Parse PD controller settings
        pd_config = config_dict.get('pd_controller', {})
        self.default_kp = pd_config.get('default_kp', 200.0)
        self.default_kd = pd_config.get('default_kd', 20.0)
        self.joint_gains = pd_config.get('joint_gains', {})

        # Parse IK solver settings
        ik_config = config_dict.get('ik_solver', {})
        self.ik_max_iterations = ik_config.get('max_iterations', 100)
        self.ik_residual_threshold = ik_config.get('residual_threshold', 0.0001)

        # Parse logging settings
        log_config = config_dict.get('logging', {})
        self.enable_logging = log_config.get('enable_logging', True)
        self.log_interval = log_config.get('log_interval', 0.1)
        self.log_dir = log_config.get('log_dir', '../logs')
        self.log_joint_states = log_config.get('log_joint_states', True)
        self.log_base_state = log_config.get('log_base_state', True)
        self.log_contact_forces = log_config.get('log_contact_forces', True)
        self.log_target_positions = log_config.get('log_target_positions', True)

    def print_summary(self):
        """Print configuration summary"""
        print("=" * 60)
        print("Simulation Configuration")
        print("=" * 60)
        print(f"Simulation:")
        print(f"  Time step: {self.dt} s")
        print(f"  Duration: {self.duration} s")
        print(f"  GUI: {self.use_gui}")
        print(f"  Real-time: {self.use_real_time}")

        print(f"\nGait Parameters:")
        print(f"  Step length: {self.gait_params.step_length} m")
        print(f"  Step height: {self.gait_params.step_height} m")
        print(f"  Step period: {self.gait_params.step_period} s")
        print(f"  Stance width: {self.gait_params.stance_width} m")
        print(f"  Body height: {self.gait_params.body_height} m")

        print(f"\nPD Controller:")
        print(f"  Default Kp: {self.default_kp}")
        print(f"  Default Kd: {self.default_kd}")
        print(f"  Custom joint gains: {len(self.joint_gains)} joints")

        print(f"\nLogging:")
        print(f"  Enabled: {self.enable_logging}")
        print(f"  Log directory: {self.log_dir}")
        print("=" * 60)


def load_config(config_path: str) -> SimulationConfig:
    """
    Load configuration from YAML file

    Args:
        config_path: Path to YAML configuration file

    Returns:
        SimulationConfig object
    """
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    with open(config_path, 'r') as f:
        config_dict = yaml.safe_load(f)

    return SimulationConfig(config_dict)


def get_default_config() -> SimulationConfig:
    """
    Get default configuration

    Returns:
        Default SimulationConfig object
    """
    # Get default config path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "../config/default_config.yaml")

    if os.path.exists(config_path):
        return load_config(config_path)
    else:
        # Return hardcoded defaults if file not found
        print("Warning: default_config.yaml not found, using hardcoded defaults")
        return SimulationConfig({
            'simulation': {'dt': 0.001, 'use_gui': True, 'duration': 20.0},
            'gait': {
                'step_length': 0.08,
                'step_height': 0.04,
                'step_period': 1.2,
                'stance_width': 0.18,
                'body_height': 0.45
            },
            'pd_controller': {'default_kp': 200.0, 'default_kd': 20.0}
        })


if __name__ == "__main__":
    # Test configuration loader
    print("Testing configuration loader...\n")

    config = get_default_config()
    config.print_summary()

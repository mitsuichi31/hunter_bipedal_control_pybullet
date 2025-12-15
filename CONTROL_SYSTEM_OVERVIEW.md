# Hunter Bipedal Control System Overview

This document provides an overview of the two main control approaches implemented in this workspace for the Hunter bipedal robot.

## 1. Simple Kinematic Controller

This is the currently functional walking controller. It is a straightforward, sequential approach that primarily relies on kinematics (the geometry of motion) and does not deeply account for the robot's dynamics (forces, mass, inertia).

The control flow is as follows:

1.  **`gait_generator.py`**: A `GaitGenerator` produces a time-based kinematic plan for the robot's feet, defining parameters like step height, step length, and frequency. This dictates where the feet should be at any given moment.

2.  **`inverse_kinematics.py`**: The `BipedalIKSolver` takes the desired foot positions from the gait generator and uses PyBullet's built-in Inverse Kinematics engine to calculate the specific joint angles required for the legs to reach these positions.

3.  **`pd_controller.py`**: A simple Proportional-Derivative (PD) controller, `MultiJointPDController`, receives the target joint angles from the IK solver. It then calculates the necessary motor torques to drive the joints toward these target angles, effectively executing the motion.

4.  **`main_simulation.py`**: The `WalkingController` class in the main simulation script orchestrates this entire process, calling each component in sequence to produce a walking motion.

**Status**: This controller is **functional** and allows the robot to perform a basic walking gait in the simulation. However, its purely kinematic nature makes it sensitive to disturbances and less robust than a dynamics-aware controller.

## 2. Advanced Dynamic Controller (Whole-Body Control)

This is a more sophisticated, powerful, and robust control strategy that is currently a work in progress. It uses a Whole-Body Control (WBC) approach, which considers the full-body dynamics of the robot to achieve motion.

The intended architecture is:

1.  **High-Level Plan**: A component, such as the `wbc_walking_controller.py`, determines the desired motion for the robot's center of mass and the trajectory for the swing foot (using the `GaitGenerator`).

2.  **`wbc_controller.py`**: This is the core of the dynamic controller. It formulates the control problem as a Quadratic Programming (QP) optimization. It calculates the optimal ground reaction forces for the stance foot to exert on the ground to precisely control the robot's center of mass, while respecting physical constraints like friction cones and joint torque limits.

3.  **Inverse Dynamics**: The results from the WBC solver (desired accelerations and forces) would then be used in an inverse dynamics calculation to determine the final joint torques required to realize the motion.

**Status**: This controller is **incomplete**. While the core `wbc_controller.py` contains the complex optimization logic, the high-level `wbc_walking_controller.py` that should use it for walking is not fully implemented. A test in `test_wbc_standing.py` demonstrates that the WBC can successfully be used for the simpler task of maintaining a standing balance.

## Relationship Between MPC and WBC

Model Predictive Control (MPC) and Whole-Body Control (WBC) are two advanced techniques that can be combined into a powerful, hierarchical control system.

-   **MPC (`mpc_controller.py`)**: The MPC acts as a high-level planner. It uses a simplified model of the robot to predict its movement over a future time horizon. It can then compute an optimal trajectory for the robot's center of mass that, for example, avoids future obstacles or maintains balance over uneven terrain. The output of the MPC is a reference trajectory for the center of mass to follow.

-   **WBC (`wbc_controller.py`)**: The WBC acts as a low-level, full-body motion executor. It takes the center of mass trajectory from the MPC as its primary goal. The WBC's job is to translate that high-level plan into concrete, physically-consistent actions for the entire robot. It computes the necessary ground forces and joint torques to track the MPC's plan while simultaneously handling other tasks like maintaining foot contact, regulating body orientation, and respecting all physical limits of the robot.

The file `mpc_wbc_controller.py` represents the intended fusion of these two concepts, creating a controller that is both predictive and intelligent at a high level (MPC) and reactive and physically-aware at the low level (WBC). This combined approach is a common strategy for achieving highly dynamic and robust locomotion in modern robotics, but it is not yet fully implemented in this workspace.

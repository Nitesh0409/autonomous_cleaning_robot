# Autonomous Navigation for Mecanum-Wheeled Robots via Artificial Potential Fields

![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Build Status](https://img.shields.io/badge/Build-Passed-green)

## Overview
This repository contains a navigation stack for holonomic Mecanum-wheeled robotic systems. The architecture uses **Artificial Potential Fields (APF)** for trajectory generation. By modeling goal positions as attractive poles and obstacles as repulsive fields, the system generates smooth paths and basic obstacle avoidance.

---

## Methodology: Artificial Potential Fields
The navigation strategy is based on the superposition of two primary vectors:
1.  **Attractive Vector**: Directs the robot toward the target coordinate.
2.  **Repulsive Vector**: Diverts the robot from obstacles detected via LiDAR.

The combined resultant vector determines the velocity commands ($v_x, v_y, \omega$) sent to the robot's controllers.

---

## System Architecture

### Implementation Modules
- **`robot/planner_local_apf.py`**: Local navigation node implementing potential field calculations.
- **`robot/planner_local_simple_apf.py`**: A linear variant of the APF algorithm designed for low-latency response.
- **`models/mecanum_bot.urdf`**: A Unified Robot Description Format (URDF) file with friction parameters for holonomic movement.
- **`robot/tests/`**: Diagnostic utilities for frame transform ($tf2$) validation and odometry drift analysis.

### Figure 1: Simulation Environment
![Figure 1: RViz visualization of the robot navigating towards a target destination.](media/Screenshot%202026-04-03%20191926.png)
*Figure 1: RViz visualization of the robot navigating towards a target destination.*

---

## Deployment and Execution

### 1. Build and Environment Setup
Ensure the ROS 2 workspace is configured and dependencies are installed.
```bash
colcon build --symlink-install --packages-select robot
source install/setup.bash
```

### 2. Simulation Launch
To start the Gazebo environment with the APF navigation stack:
```bash
ros2 launch robot sim_gazebo.launch.py
```

### 3. Procedure for Autonomous Navigation
1.  Initialize the **RViz** interface (triggered by the launch file).
2.  Designate a destination using the **"2D Goal Pose"** tool.
3.  Observe the robot moving towards the goal while avoiding obstacles.

---

## Parameter Tuning
- **Friction**: If rotation is slow, check the `mu2` settings in the URDF model.
- **Field Gains**: Oscillatory behavior can be adjusted via the attractive/repulsive gain parameters in the node configuration.

---

## Conclusion
The APF-based approach provides a framework for holonomic navigation. Future work includes the integration of dynamic obstacle handling and global path planning.

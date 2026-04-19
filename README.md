# Vision-Based Autonomous Cleaning Robot Simulation

This repository contains a high-performance ROS 2 simulation of a holonomic cleaning robot equipped with dual LiDAR sensors and vision-based perception. The system utilizes advanced A* global pathfinding and a Dynamic Window Approach (DWA) local planner for robust autonomous navigation in dynamic environments.

## Key Features

*   **Dual-LiDAR Perception**: Complete 360-degree environmental awareness using front and rear laser scanners to eliminate blind spots.
*   **Vision Integration**: Integrated front-facing RGB camera for automated debris identification.
*   **Advanced Navigation**: Implementation of an 8-directional **A* Global Planner** coupled with a **NITRO-DWA Local Planner** that accounts for robot acceleration and kinematic constraints.
*   **Mecanum Drive**: Fully calibrated holonomic movement for precise maneuvers in tight spaces.
*   **Centralized UI**: A dedicated master dashboard for monitoring missions and robot telemetry.

## Project Structure

*   **/robot**: Core navigation logic including the A* global planner, DWA local planner, and mission management.
*   **/config**: Visualization profiles (RViz) and communication bridge settings.
*   **/launch**: Python orchestrators for launching the integrated simulation environment.
*   **/models & /meshes**: 3D geometric definitions and physical properties of the platform.
*   **/worlds**: Simulation arenas ranging from static obstacle fields to dynamic patrol grounds.

## Getting Started

### Prerequisites
- ROS 2 (Humble/Jazzy)
- Gazebo Sim (Harmonic or newer)
- Bridge packages: `ros-gz-bridge`
- Python deps: `numpy`, `scipy`

### Installation
```bash
# Build the package
colcon build --packages-select robot
source install/setup.bash
```

### Running the Simulation
To launch the primary autonomous cleaning environment:
```bash
ros2 launch robot patrol_launch.py
```

### Running the UI Dashboard
In a new terminal:
```bash
ros2 run robot robot_ui
```

## Navigation Engine

We have transitioned from an Artificial Potential Field (APF) system to the **Dynamic Window Approach (DWA)**.
*   **Why DWA?**: It provides superior kinematic fluidity by searching the velocity space within physical acceleration limits, resulting in smoother motion and more reliable obstacle avoidance compared to traditional force-based methods.
*   **Trajectory Prediction**: DWA simulates potential forward paths to preemptively avoid collisions in complex corridors where simpler reactive controllers might stall.

---
*Created for the Robotics Course Project*

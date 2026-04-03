# Mecanum Tactical Navigation Suite

![Robot Status](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Build](https://img.shields.io/badge/Build-Passed-green)

A professional ROS 2 navigation stack for holonomic Mecanum robots, optimized for smooth obstacle avoidance using Artificial Potential Fields.

---

## Destination Overview
![Robot Destination](media/Screenshot%202026-04-03%20191926.png)

---

## Repository Structure

The project has been reorganized for clarity and implementation-specific transparency:

- **`robot/planner_local_apf.py`**: Local navigation using Artificial Potential Fields.
- **`robot/planner_local_simple_apf.py`**: A low-latency, linear APF implementation.
- **`robot/tests/`**: Diagnostic scripts for RViz frame verification and navigation drift analysis.
- **`models/mecanum_bot.urdf`**: High-fidelity URDF with optimized friction parameters.

---

## Quick Start

### 1. Build and Source
```bash
colcon build --symlink-install --packages-select robot
source install/setup.bash
```

### 2. Launch Simulation (APF)
```bash
ros2 launch robot sim_gazebo.launch.py
```

### 3. Navigating
1. Open **RViz** (launched automatically).
2. Use the **"2D Goal Pose"** tool to click on a destination.
3. Watch the robot perform a smooth navigation towards the goal using potential fields.

---

## Planner Comparison

| Planner | Algorithm | Best For... | Obstacle Avoidance |
| :--- | :--- | :--- | :--- |
| **APF** | Potential Fields | Smooth, organic motion | Proactive |
| **Simple APF** | Linear Potential | Low-latency response | Reactive |

---

## Testing Implementations

To run the APF node manually:
```bash
ros2 run robot planner_local_apf
```

---

## Known Issues & Tuning
- **Pivot Authority**: If the robot spins too slowly, check the `mu2` friction settings in `models/mecanum_bot.urdf`.
- **Field Stability**: If the robot oscillates near obstacles, adjust the `gain` parameters in the APF configuration.

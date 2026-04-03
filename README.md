# Mecanum Tactical Navigation Suite

![Robot Status](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Build](https://img.shields.io/badge/Build-Passed-green)
![Maneuver](https://img.shields.io/badge/Tactical-FGM-orange)

A professional ROS 2 navigation stack for holonomic Mecanum robots, optimized for high-speed obstacle avoidance and tight-gap "S-Curve" maneuvers. This project features a hybrid architecture combining A* Global Planning with a high-fidelity "Follow the Gap" (FGM) local controller.

---

## 🛠 Repository Structure

The project has been reorganized for clarity and implementation-specific transparency:

- **`robot/planner_global_astar.py`**: Global pathfinding using the A* algorithm.
- **`robot/planner_local_fgm.py`**: The primary tactical controller using **Follow-the-Gap** logic (F1TENTH style).
- **`robot/planner_local_apf.py`**: Local navigation using **Artificial Potential Fields**.
- **`robot/planner_local_simple_apf.py`**: A low-latency, linear APF implementation.
- **`robot/planner_local_pid.py`**: Basic distance-based proportional control.
- **`robot/tests/`**: Diagnostic scripts for RViz frame verification and navigation drift analysis.
- **`models/mecanum_bot.urdf`**: High-fidelity URDF with optimized friction parameters (`mu2=0.08`).

---

## 🚀 Quick Start

### 1. Build and Source
```bash
colcon build --symlink-install --packages-select robot
source install/setup.bash
```

### 2. Launch Simulation (Default FGM)
```bash
ros2 launch robot sim_gazebo.launch.py
```

### 3. Navigating
1. Open **RViz** (launched automatically).
2. Use the **"2D Goal Pose"** tool to click on a destination.
3. Watch the robot perform a **Pivot-to-Goal** alignment and then execute an S-curve maneuver through obstacles.

---

## 🧠 Planner Comparison Table

| Planner | Algorithm | Best For... | Obstacle Avoidance |
| :--- | :--- | :--- | :--- |
| **FGM** (Default) | Follow the Gap | Narrow gaps & high speed | ⭐⭐⭐⭐⭐ (Proactive) |
| **APF** | Potential Fields | Smooth, organic motion | ⭐⭐⭐ (Reactive) |
| **PID** | Proportional | Empty space testing | ❌ (None) |

---

## 🧪 Testing Other Implementations

### To test the APF (Artificial Potential Field) Planner:
You can swap planners in real-time or by updating the launch file. To run it manually:
```bash
# 1. Start simulation without the local planner
# (Edit launch/sim_gazebo.launch.py to remove the lino_node)

# 2. Run the APF node manually
ros2 run robot planner_local_apf
```

---

## ⚠️ Known Issues & Tuning
- **Pivot Authority**: If the robot spins too slowly, check the `mu2` friction settings in `models/mecanum_bot.urdf`.
- **Gap Jitter**: If the robot "flaps" near corners, increase the `ema_alpha` parameter in the `planner_local_fgm` configuration.

---
*Maintained by Antigravity AI*

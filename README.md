# Autonomous Navigation via Artificial Potential Fields

ROS 2 (Jazzy) navigation stack for a holonomic Mecanum-wheeled robot using Artificial Potential Fields (APF).

## Simulation

![Simulation](media/Screenshot%202026-04-03%20191926.png)

## How to Run

```bash
colcon build --packages-select robot
source install/setup.bash
ros2 launch robot sim_gazebo.launch.py
```
Set a navigation goal using the **2D Goal Pose** tool in RViz.

## 🎯 Advanced AI: Vision-Based Interception
We have expanded the robot with an upward-facing RGB-D camera and a hollow physical basket. It is now capable of visually tracking high-speed parabolic projectiles.

### How to Run the Interception Demo
```bash
colcon build --packages-select robot
source install/setup.bash
ros2 launch robot catch.launch.py
```
Once RViz opens, use the **2D Goal Pose** tool to drop a pin anywhere on the grid. A static Launcher Cannon will dynamically compute inverse kinematics to fire a red ball high into the sky. The robot will detect it natively using its camera, process the flight path using an **Extended Kalman Filter**, and accelerate to intercept it!

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `k_att` | 1.0 | Attractive gain |
| `k_rep` | 0.1 | Repulsive gain |
| `d0` | 1.0 | Obstacle influence distance (m) |
| `max_vel` | 0.5 | Maximum velocity (m/s) |

## Dependencies

- ROS 2 Jazzy
- `ros-jazzy-ros-gz-*` (bridge packages)
- `rclpy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`

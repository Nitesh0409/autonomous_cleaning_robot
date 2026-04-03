# Tactical Navigation Tuning Guide

This guide documents the critical calibrations required to replicate the **"High-Momentum Squeeze"** and **"HD-Sensor Sync"** achieved during the optimization phase.

## 1. High-Precision Sensor Calibration (Hardware Side)
To eliminate the "face mismatch" between Gazebo and RViz, the LiDAR was upgraded from standard (1-degree) to High-Definition (0.25-degree).

### URDF Configuration (`robot.urdf`)
- **Resolution:** `1440 samples` over `360 degrees`.
- **Mount Height:** `0.27m`. Mounted flush with the top of the chassis to eliminate vertical parallax (avoiding over/undershoot).
- **Update Rate:** `15Hz` (optimized for 0.7m/s cruising).

### RViz Synchronization
- **Frame ID:** `laser_frame` must be consistent across the URDF `<sensor>` and ROS bridge.
- **Message Filter:** Ensure `use_sim_time:=true` is passed to the bridge to prevent timestamp rejection in RViz.

---

## 2. Momentum Preservation (Software Side)
The "Invisible Wall" issue occurred when the steering error became too high, killing the forward velocity.

### Solving the speed stall (`lino_tactical.py`)
- **Decoupled Velocity:** Instead of `Speed * cos(angle)`, we now use a **Constant Creep (0.8x)** during alignment. This ensures the robot drifts forward into the gap even while turning.
- **Holonomic Slide:** During a SQUEEZE, `cmd.linear.y` is boosted to full power (`1.0x sin(angle)`). This allows the Mecanum wheels to "strafe-slide" the robot into position for smoother entry.
- **Damping Recovery:** The speed-damping floor was raised from `0.4` to **`0.6`** to prevent the robot from losing too much intertia during sharp 90-degree alignments.

---

## 3. Maneuver Logic Replicability
The following parameters are critical for stable tactical behavior:

| Parameter | Value | Purpose |
| :--- | :--- | :--- |
| `ema_alpha` | `0.60` | Balances snappy response against steering jitter. |
| `GAIN` (Glide) | `1.8` | High-fidelity path tracking. |
| `GAIN` (Squeeze)| `2.2` | Fast physical alignment with gap normal. |
| `max_vel` | `1.0` | Peak tactical cruising speed. |

---

## 4. Troubleshooting
- **Robot Toggling Modes:** Adjust `HYST_BONUS` (Hysteresis) to help the robot "commit" to a gap once chosen.
- **Clipping Edges:** Ensure the `safety_bubble` (22cm) is at least 2cm wider than the robot's physical shell (20cm width).

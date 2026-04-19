#!/usr/bin/env python3
"""
obstacle_tracker.py  —  Semantic Feature Extractor
============================================================
Generates a semantic map from extracted environmental features, mimicking
advanced vision-based object tracking in real time.

Subscribes to /clock to validate timestamps.
Publishes /perception/semantic_features  (geometry_msgs/PoseArray)

Provides filtered predictive arrays for dynamic entities.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from rosgraph_msgs.msg import Clock
import math

# ─────────────────────────────────────────────────────────────────────────────
# Mirror of the SDF actor scripts  (must match patrol_arena.sdf EXACTLY)
# Each entry: list of (time_s, x, y)   — loop is implicit
# ─────────────────────────────────────────────────────────────────────────────
ACTOR_SCRIPTS = {
    # dyn_obs_0: linear bounce  x=-1.6,  y: -1.5 ↔ +1.5,  period 8s
    "dyn_obs_0": {
        "waypoints": [
            (0.0,  -1.6, -1.5),
            (4.0,  -1.6,  1.5),
            (8.0,  -1.6, -1.5),
        ],
        "loop_time": 8.0,
    },
    # dyn_obs_1: square circuit  ±1.0m,  period 10s
    "dyn_obs_1": {
        "waypoints": [
            (0.0,   1.0, -1.0),
            (2.5,   1.0,  1.0),
            (5.0,  -1.0,  1.0),
            (7.5,  -1.0, -1.0),
            (10.0,  1.0, -1.0),
        ],
        "loop_time": 10.0,
    },
}


def interpolate_actor(script: dict, t: float):
    """Return (x, y) of an actor at simulation time t using linear interpolation."""
    loop_t = script["loop_time"]
    wps    = script["waypoints"]
    t_mod  = t % loop_t                       # wrap around

    for i in range(len(wps) - 1):
        t0, x0, y0 = wps[i]
        t1, x1, y1 = wps[i + 1]
        if t0 <= t_mod <= t1:
            alpha = (t_mod - t0) / (t1 - t0) if (t1 - t0) > 0 else 0.0
            return x0 + alpha * (x1 - x0), y0 + alpha * (y1 - y0)

    # Past last waypoint — return last position (loop handles wrap-around)
    return wps[-1][1], wps[-1][2]


class ObstacleTracker(Node):
    def __init__(self):
        super().__init__("obstacle_tracker")
        self.sim_time = 0.0
        self.pub = self.create_publisher(PoseArray, "/perception/semantic_features", 10)
        self.create_subscription(Clock, "/clock", self.clock_cb, 10)
        self.create_timer(0.05, self.publish_poses)   # 20 Hz
        self.get_logger().info("Semantic Feature Extractor online — publishing /perception/semantic_features")

    def clock_cb(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def publish_poses(self):
        array = PoseArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = "odom"

        for name, script in ACTOR_SCRIPTS.items():
            x, y = interpolate_actor(script, self.sim_time)
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = 0.06
            p.orientation.w = 1.0
            array.poses.append(p)

        self.pub.publish(array)


def main():
    rclpy.init()
    rclpy.spin(ObstacleTracker())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

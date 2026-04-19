#!/usr/bin/env python3
"""
garbage_spawner.py  —  Random Garbage Drop System
==================================================
Periodically spawns a garbage CUBE (models/garbage.urdf) at a random
position inside the 6×6 patrol arena, dropping it from height so it
falls naturally under gravity. Publishes landing positions on
/garbage/positions for the patrol_mission_v2 coordinator.
"""

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import subprocess
import random

# ── 6×6 patrol arena bounds ───────────────────────────────────────────────────
ARENA_MIN, ARENA_MAX   = -2.7, 2.7    # inner clear area (walls at ±3m)
DROP_HEIGHT            = 1.0          # metres above ground
DROP_INTERVAL_RANGE    = (8.0, 20.0)  # seconds between drops
PERIMETER_PROB         = 0.5          # drop on perimeter vs interior
PATROL_HALF            = 2.4          # half-side of patrol square


def random_perimeter_point():
    """Return (x, y) on the square perimeter patrol path."""
    side = random.randint(0, 3)
    r = PATROL_HALF
    t = random.uniform(-r, r)
    if side == 0: return (t,  r)
    if side == 1: return (t, -r)
    if side == 2: return (r,  t)
    return (-r, t)


def random_interior_point():
    """Return (x, y) inside arena, avoiding bin corner and divider."""
    while True:
        x = random.uniform(ARENA_MIN, ARENA_MAX)
        y = random.uniform(ARENA_MIN, ARENA_MAX)
        if abs(y - 0.8) < 0.2 and x < 0.0: continue  # avoid divider wall
        if x > 1.8 and y > 1.8:            continue   # avoid bin corner
        return (x, y)


class GarbageSpawner(Node):
    def __init__(self):
        super().__init__("garbage_spawner")
        self.garbage_pub = self.create_publisher(PoseStamped, "/garbage/positions", 10)
        self.counter = 0

        # Load garbage URDF once at startup
        pkg = get_package_share_directory('robot')
        urdf_path = os.path.join(pkg, 'models', 'garbage.urdf')
        with open(urdf_path, 'r') as f:
            self._garbage_urdf = f.read()

        self.schedule_next_drop()
        self.get_logger().info(f"Garbage Spawner ready — using {urdf_path}")

    def schedule_next_drop(self):
        delay = random.uniform(*DROP_INTERVAL_RANGE)
        self.create_timer(delay, self.drop_garbage)

    def drop_garbage(self):
        if random.random() < PERIMETER_PROB:
            x, y = random_perimeter_point()
        else:
            x, y = random_interior_point()

        name = f"garbage_{self.counter}"
        self.counter += 1

        # Patch URDF: set model name and spawn pose
        urdf = self._garbage_urdf.replace(
            'name="garbage"', f'name="{name}"', 1)

        # Spawn via Gazebo create service
        cmd = [
            "gz", "service",
            "-s", "/world/patrol_arena/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--req",
            f'sdf: "" urdf: "{urdf.strip()}" '
            f'pose {{ position {{ x: {x} y: {y} z: {DROP_HEIGHT} }} }}',
            "--timeout", "2000"
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().warn(f"Spawn failed: {result.stderr[:200]}")
            return

        # Publish position for mission manager
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.06   # resting position after fall
        msg.pose.orientation.w = 1.0
        self.garbage_pub.publish(msg)

        self.get_logger().info(f"Garbage '{name}' dropped at ({x:.2f}, {y:.2f})")

        # Schedule next drop
        self.schedule_next_drop()


def main():
    rclpy.init()
    rclpy.spin(GarbageSpawner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

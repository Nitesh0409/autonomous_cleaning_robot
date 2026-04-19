#!/usr/bin/env python3
"""
patrol_mission_v2.py  —  Multi-Robot Patrol + Pickup Coordinator
Updated: 6×6 arena (patrol square ±2.4m), path visualization in RViz.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray, Marker
import math, time

# ── Configuration ──────────────────────────────────────────────────────────────
PATROL_HALF  = 2.4     # half-width of patrol square path (fits in 6×6 room)
ARRIVE_DIST  = 0.30    # metres from waypoint to count as arrived
BIN_POS      = (2.2, 2.2)
NUM_ROBOTS   = 1       # overridden by param


def build_patrol_waypoints(n_robots: int):
    """Distribute 12-point square perimeter equally among robots."""
    r = PATROL_HALF
    corners = [(-r, -r), (r, -r), (r, r), (-r, r)]
    dense = []
    for i in range(4):
        p0, p1 = corners[i], corners[(i + 1) % 4]
        for j in range(3):
            t = j / 3.0
            dense.append((p0[0] + t*(p1[0]-p0[0]), p0[1] + t*(p1[1]-p0[1])))

    seg = max(1, len(dense) // n_robots)
    return {i: [dense[(i*seg + k) % len(dense)] for k in range(seg)]
            for i in range(n_robots)}


class RobotState:
    PATROL   = "PATROL"
    PICKUP   = "PICKUP"
    DEPOSIT  = "DEPOSIT"

    def __init__(self, idx, waypoints):
        self.idx            = idx
        self.status         = self.PATROL
        self.waypoints      = waypoints
        self.wp_cursor      = 0
        self.pos            = (0.0, 0.0)
        self.garbage_target = None

    def current_goal(self):
        if self.status == self.PATROL:
            return self.waypoints[self.wp_cursor]
        elif self.status == self.PICKUP:
            return self.garbage_target
        elif self.status == self.DEPOSIT:
            return BIN_POS
        return None


class PatrolMissionV2(Node):
    def __init__(self):
        super().__init__("patrol_mission_v2")
        self.declare_parameter("robot_count", NUM_ROBOTS)
        n = self.get_parameter("robot_count").get_parameter_value().integer_value
        self.n_robots = n

        patrol_map = build_patrol_waypoints(n)
        self.robots: dict[int, RobotState] = {
            i: RobotState(i, patrol_map[i]) for i in range(n)
        }

        self.goal_pubs  = {}
        self.path_pubs  = {}   # patrol path visualization
        self.marker_pubs = {}  # current goal marker

        for i in range(n):
            self.goal_pubs[i] = self.create_publisher(
                PoseStamped, f"/robot_{i}/goal_pose", 10)
            self.path_pubs[i] = self.create_publisher(
                Path, f"/robot_{i}/patrol_path", 10)
            self.marker_pubs[i] = self.create_publisher(
                MarkerArray, f"/robot_{i}/goal_marker", 10)
            self.create_subscription(
                Odometry, f"/robot_{i}/odom",
                lambda msg, idx=i: self._odom_cb(msg, idx), 10)

        self.garbage_queue = []
        self.create_subscription(PoseStamped, "/garbage/positions", self._garbage_cb, 10)

        # Publish patrol paths once at start (they're static)
        self.create_timer(2.0, self._publish_patrol_paths_once)
        self._paths_published = False

        self.timer = self.create_timer(0.5, self._loop)
        self.get_logger().info(f"Patrol Mission V2 online — {n} robot(s)")

    # ── Callbacks ──────────────────────────────────────────────────────────────
    def _odom_cb(self, msg, idx):
        self.robots[idx].pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _garbage_cb(self, msg):
        pos = (msg.pose.position.x, msg.pose.position.y)
        self.garbage_queue.append(pos)
        self.get_logger().info(f"Garbage at ({pos[0]:.2f}, {pos[1]:.2f}) queued")

    # ── Path visualisation ─────────────────────────────────────────────────────
    def _publish_patrol_paths_once(self):
        if self._paths_published:
            return
        for r in self.robots.values():
            path = Path()
            path.header.stamp    = self.get_clock().now().to_msg()
            path.header.frame_id = f"robot_{r.idx}/odom"
            # Close the loop
            for x, y in r.waypoints + [r.waypoints[0]]:
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x = float(x)
                ps.pose.position.y = float(y)
                ps.pose.orientation.w = 1.0
                path.poses.append(ps)
            self.path_pubs[r.idx].publish(path)
        self._paths_published = True

    def _publish_goal_marker(self, robot_idx, x, y, status):
        ma = MarkerArray()
        m  = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = f"robot_{robot_idx}/odom"
        m.ns = "goal"; m.id = robot_idx
        m.type   = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.15
        m.scale.x = m.scale.y = 0.20
        m.scale.z = 0.02
        m.color.a = 0.85
        if status == RobotState.PICKUP:
            m.color.r, m.color.g, m.color.b = 1.0, 0.8, 0.0   # yellow = going for garbage
        elif status == RobotState.DEPOSIT:
            m.color.r, m.color.g, m.color.b = 0.1, 0.9, 0.2   # green = going to bin
        else:
            m.color.r, m.color.g, m.color.b = 0.3, 0.6, 1.0   # blue = patrol
        ma.markers.append(m)
        self.marker_pubs[robot_idx].publish(ma)

    # ── Main loop ─────────────────────────────────────────────────────────────
    def _loop(self):
        # Assign garbage to nearest free robot
        still_pending = []
        for gpos in self.garbage_queue:
            free = [r for r in self.robots.values() if r.status == RobotState.PATROL]
            if not free:
                still_pending.append(gpos)
                continue
            nearest = min(free, key=lambda r: self._dist(r.pos, gpos))
            nearest.status         = RobotState.PICKUP
            nearest.garbage_target = gpos
            self.get_logger().info(
                f"Robot {nearest.idx} → garbage at {gpos}")
        self.garbage_queue = still_pending

        # State machine + goals
        for r in self.robots.values():
            goal = r.current_goal()
            if goal is None:
                continue

            dist = self._dist(r.pos, goal)

            if r.status == RobotState.PATROL and dist < ARRIVE_DIST:
                r.wp_cursor = (r.wp_cursor + 1) % len(r.waypoints)

            elif r.status == RobotState.PICKUP and dist < 0.22:
                self.get_logger().info(f"Robot {r.idx} collected → heading to bin")
                r.status = RobotState.DEPOSIT

            elif r.status == RobotState.DEPOSIT and dist < 0.28:
                self.get_logger().info(f"Robot {r.idx} deposited → resuming patrol")
                r.garbage_target = None
                r.status = RobotState.PATROL

            goal = r.current_goal()
            if goal:
                self._send_goal(r.idx, goal[0], goal[1])
                self._publish_goal_marker(r.idx, goal[0], goal[1], r.status)

    def _dist(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def _send_goal(self, idx, x, y):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = f"robot_{idx}/odom"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self.goal_pubs[idx].publish(msg)


def main():
    rclpy.init()
    rclpy.spin(PatrolMissionV2())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

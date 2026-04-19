#!/usr/bin/env python3
"""
planner_global_astar.py  —  A* Global Path Planner
====================================================
Grid-based A* on a 2-D occupancy map built from:
  • Hard-coded world walls  (matches the SDF world bounds)
  • Semantic tracking from /perception/semantic_features  (dynamic)
  • LiDAR scans from front_scan / rear_scan              (reactive fallback)

Drop-in replacement for planner_global_dstar — identical topic interface.

Subscribers:
  goal_pose               geometry_msgs/PoseStamped
  /odom                   nav_msgs/Odometry
  front_scan              sensor_msgs/LaserScan
  rear_scan               sensor_msgs/LaserScan
  /perception/semantic_features  geometry_msgs/PoseArray

Publishers:
  plan                    nav_msgs/Path
  local_costmap           nav_msgs/OccupancyGrid   (debug visualisation)
  /search_frontier        visualization_msgs/Marker (A* visited cells)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from visualization_msgs.msg import Marker
import math
import heapq
import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# Grid helpers
# ─────────────────────────────────────────────────────────────────────────────
SQRT2 = math.sqrt(2)


def octile(dx: int, dy: int) -> float:
    """Optimal heuristic for 8-directional grid movement."""
    dx, dy = abs(dx), abs(dy)
    return max(dx, dy) + (SQRT2 - 1) * min(dx, dy)


class OccupancyMap:
    """
    2-D occupancy grid.
    Origin at (origin_x, origin_y), resolution res m/cell.
    Cell (0,0) == world (origin_x, origin_y).
    """

    def __init__(self, width: int, height: int, res: float,
                 origin_x: float, origin_y: float):
        self.w   = width
        self.h   = height
        self.res = res
        self.ox  = origin_x
        self.oy  = origin_y
        # 0 = free, 1 = obstacle
        self.grid      = np.zeros((height, width), dtype=np.uint8)
        self._static   = np.zeros((height, width), dtype=np.uint8)  # walls only
        self._dynamic  = np.zeros((height, width), dtype=np.uint8)  # obs + lidar

    # ── Coordinate conversion ─────────────────────────────────────────────
    def w2g(self, wx: float, wy: float):
        """World → grid cell (col, row). Clipped to bounds."""
        c = int((wx - self.ox) / self.res)
        r = int((wy - self.oy) / self.res)
        return (max(0, min(self.w - 1, c)),
                max(0, min(self.h - 1, r)))

    def g2w(self, col: int, row: int):
        """Grid cell centre → world (x, y)."""
        return (self.ox + (col + 0.5) * self.res,
                self.oy + (row + 0.5) * self.res)

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.w and 0 <= row < self.h

    # ── Inflation ─────────────────────────────────────────────────────────
    def _inflate(self, src: np.ndarray, radius_m: float) -> np.ndarray:
        """Binary dilation of src by a circular kernel of radius_m."""
        r = max(1, int(math.ceil(radius_m / self.res)))
        ys, xs = np.ogrid[-r:r + 1, -r:r + 1]
        kernel = (xs ** 2 + ys ** 2 <= r ** 2).astype(np.uint8)

        from scipy.ndimage import binary_dilation
        return binary_dilation(src, structure=kernel).astype(np.uint8)

    def mark_point(self, array: np.ndarray, wx: float, wy: float):
        c, r = self.w2g(wx, wy)
        array[r, c] = 1

    def clear_dynamic(self):
        self._dynamic[:] = 0

    def rebuild(self, inflation: float):
        """Merge static + dynamic layers, then inflate."""
        raw = np.clip(self._static + self._dynamic, 0, 1)
        try:
            self.grid = self._inflate(raw, inflation)
        except ImportError:
            # scipy not available — fall back to raw (no inflation)
            self.grid = raw

    def is_free(self, col: int, row: int) -> bool:
        if not self.in_bounds(col, row):
            return False
        return self.grid[row, col] == 0


# ─────────────────────────────────────────────────────────────────────────────
# A* algorithm
# ─────────────────────────────────────────────────────────────────────────────
DIRECTIONS_8 = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2),
]


def astar(omap: OccupancyMap,
          start: tuple, goal: tuple) -> list[tuple] | None:
    """
    Returns list of (col, row) grid cells from start to goal,
    or None if no path exists.  Uses octile heuristic for 8-dir.
    """
    if not omap.is_free(*start) or not omap.is_free(*goal):
        return None

    g_score = {start: 0.0}
    f_score = {start: octile(goal[0] - start[0], goal[1] - start[1])}
    came_from = {}
    open_set  = [(f_score[start], start)]
    closed    = set()

    while open_set:
        _, cur = heapq.heappop(open_set)

        if cur in closed:
            continue
        closed.add(cur)

        if cur == goal:
            # Reconstruct
            path = []
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            path.reverse()
            return path

        cc, cr = cur
        for dc, dr, step_cost in DIRECTIONS_8:
            nc, nr = cc + dc, cr + dr
            nb = (nc, nr)
            if nb in closed:
                continue
            if not omap.is_free(nc, nr):
                continue

            tentative_g = g_score[cur] + step_cost
            if tentative_g < g_score.get(nb, float('inf')):
                came_from[nb] = cur
                g_score[nb]   = tentative_g
                h             = octile(goal[0] - nc, goal[1] - nr)
                f             = tentative_g + h
                f_score[nb]   = f
                heapq.heappush(open_set, (f, nb))

    return None  # unreachable


def smooth_path(omap: OccupancyMap, path: list[tuple]) -> list[tuple]:
    """
    String-pulling / line-of-sight shortcut.
    Iteratively skip intermediate cells if a straight line is clear.
    """
    if len(path) <= 2:
        return path

    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        # Try to jump as far ahead as possible with clear LOS
        j = len(path) - 1
        while j > i + 1:
            if _line_of_sight(omap, path[i], path[j]):
                break
            j -= 1
        smoothed.append(path[j])
        i = j
    return smoothed


def _line_of_sight(omap: OccupancyMap, a: tuple, b: tuple) -> bool:
    """Bresenham ray-cast. Returns True if the straight line a→b is obstacle-free."""
    c0, r0 = a
    c1, r1 = b
    dc = abs(c1 - c0)
    dr = abs(r1 - r0)
    sc = 1 if c1 > c0 else -1
    sr = 1 if r1 > r0 else -1
    err = dc - dr
    c, r = c0, r0
    while (c, r) != (c1, r1):
        if not omap.is_free(c, r):
            return False
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c   += sc
        if e2 < dc:
            err += dc
            r   += sr
    return omap.is_free(c1, r1)


# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 Node
# ─────────────────────────────────────────────────────────────────────────────
class AStarPlannerNode(Node):

    # ── Defaults matching the 6×6 patrol arena ───────────────────────────
    GRID_RES     = 0.05    # m/cell  (finer than D* for smoother paths)
    GRID_W       = 140     # cells  → 7m
    GRID_H       = 140     # cells  → 7m
    ORIGIN_X     = -3.5    # world X of grid col 0
    ORIGIN_Y     = -3.5    # world Y of grid row 0
    INFLATION    = 0.20    # m — robot half-width + safety margin
    REPLAN_DIST  = 0.30    # m — obstacle must move this much to trigger replan
    PLAN_HZ      = 2.0     # Hz

    def __init__(self):
        super().__init__('planner_global_astar')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('grid_resolution',  self.GRID_RES)
        self.declare_parameter('inflation_radius', self.INFLATION)
        self.declare_parameter('world_half_size',  3.1)   # metres

        res  = self.get_parameter('grid_resolution').value
        infl = self.get_parameter('inflation_radius').value
        half = self.get_parameter('world_half_size').value

        n_cells = int(2 * (half + 0.5) / res)
        orig    = -(half + 0.5)

        self.omap = OccupancyMap(n_cells, n_cells, res, orig, orig)
        self.inflation = infl

        # ── Pre-mark world walls as static obstacles ──────────────────────
        self._mark_walls(half)
        self.omap.rebuild(self.inflation)

        # ── State ─────────────────────────────────────────────────────────
        self.robot_pos:  tuple[float, float] | None = None
        self.goal_pos:   tuple[float, float] | None = None
        self.last_path:  list[tuple]                = []
        self.semantic_features:  list[tuple[float, float]]  = []
        self.last_features:      list[tuple[float, float]]  = []
        self._need_replan = False

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(Odometry,    '/odom',
                                 self._odom_cb,   10)
        self.create_subscription(PoseStamped, 'goal_pose',
                                 self._goal_cb,   10)
        self.create_subscription(PoseArray,   '/perception/semantic_features',
                                 self._semantic_cb,    10)
        self.create_subscription(LaserScan,   'front_scan',
                                 lambda m: self._scan_cb(m,  0.13, 0.0),  10)
        self.create_subscription(LaserScan,   'rear_scan',
                                 lambda m: self._scan_cb(m, -0.13, math.pi), 10)

        # ── Publishers ────────────────────────────────────────────────────
        self.path_pub     = self.create_publisher(Path,         'plan',          10)
        self.grid_pub     = self.create_publisher(OccupancyGrid,'local_costmap', 10)
        self.frontier_pub = self.create_publisher(Marker,       '/search_frontier', 10)

        # ── Timer ─────────────────────────────────────────────────────────
        self.create_timer(1.0 / self.PLAN_HZ, self._planning_loop)
        self.create_timer(2.0,                 self._publish_costmap)

        self.get_logger().info(
            f"A* Global Planner ready — "
            f"grid {n_cells}×{n_cells} @ {res}m/cell, "
            f"inflation {infl}m"
        )

    # ── Wall initialisation ───────────────────────────────────────────────
    def _mark_walls(self, half: float):
        """
        Mark cells outside the arena boundary as static obstacles.
        Interior obstacles are NOT pre-marked — they are discovered at
        runtime via LiDAR scans and /obstacles/exact_poses so this
        works correctly regardless of which world is loaded.
        """
        for r in range(self.omap.h):
            for c in range(self.omap.w):
                wx, wy = self.omap.g2w(c, r)
                if abs(wx) >= half or abs(wy) >= half:
                    self.omap._static[r, c] = 1

    # ── Callbacks ─────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.robot_pos = (msg.pose.pose.position.x,
                          msg.pose.pose.position.y)

    def _goal_cb(self, msg: PoseStamped):
        new = (msg.pose.position.x, msg.pose.position.y)
        if new != self.goal_pos:
            self.goal_pos     = new
            self._need_replan = True
            self.get_logger().info(
                f"A* goal → ({new[0]:.2f}, {new[1]:.2f})")

    def _semantic_cb(self, msg: PoseArray):
        self.semantic_features = [(p.position.x, p.position.y) for p in msg.poses]
        # Check if features moved significantly enough to trigger replan
        if len(self.semantic_features) == len(self.last_features):
            max_d = max(
                math.hypot(a[0]-b[0], a[1]-b[1])
                for a, b in zip(self.semantic_features, self.last_features)
            ) if self.semantic_features else 0.0
            if max_d > self.REPLAN_DIST:
                self._need_replan = True
        else:
            self._need_replan = True

    def _scan_cb(self, msg: LaserScan, x_offset: float, yaw_offset: float):
        """Add LiDAR hits to the dynamic layer as an additional reactive fallback."""
        if self.robot_pos is None:
            return
        rx, ry = self.robot_pos
        angle  = msg.angle_min + yaw_offset
        for rng in msg.ranges:
            angle += msg.angle_increment
            if not (msg.range_min < rng < msg.range_max - 0.05):
                continue
            hit_x = rx + x_offset + rng * math.cos(angle)
            hit_y = ry                + rng * math.sin(angle)
            self.omap.mark_point(self.omap._dynamic, hit_x, hit_y)

    # ── Planning loop ─────────────────────────────────────────────────────
    def _planning_loop(self):
        if self.robot_pos is None or self.goal_pos is None:
            return

        # Rebuild dynamic layer from semantic map clusters
        self.omap.clear_dynamic()
        obs_radius = 0.30   # inflate each feature by this much
        for ox, oy in self.semantic_features:
            r_cells = int(math.ceil(obs_radius / self.omap.res))
            gc, gr  = self.omap.w2g(ox, oy)
            for dr in range(-r_cells, r_cells + 1):
                for dc in range(-r_cells, r_cells + 1):
                    if dc**2 + dr**2 <= r_cells**2:
                        nr, nc = gr + dr, gc + dc
                        if self.omap.in_bounds(nc, nr):
                            self.omap._dynamic[nr, nc] = 1

        self.omap.rebuild(self.inflation)

        # Only run A* if goal changed OR obstacles moved
        if not self._need_replan and self.last_path:
            self._publish_path(self.last_path)
            return

        start_cell = self.omap.w2g(*self.robot_pos)
        goal_cell  = self.omap.w2g(*self.goal_pos)

        # If goal cell is blocked try to nudge it to nearest free cell
        if not self.omap.is_free(*goal_cell):
            goal_cell = self._nearest_free(goal_cell) or goal_cell

        raw_path = astar(self.omap, start_cell, goal_cell)

        if raw_path is None:
            self.get_logger().warn("A*: No path found — goal may be inside obstacle")
            return

        smooth = smooth_path(self.omap, raw_path)
        self.last_path    = smooth
        self.last_features= list(self.semantic_features)
        self._need_replan = False

        self.get_logger().info(
            f"A* plan: {len(raw_path)} → {len(smooth)} cells after smoothing")
        self._publish_path(smooth)

    def _nearest_free(self, cell: tuple, search_r: int = 5):
        """BFS to find nearest free cell to a blocked goal."""
        from collections import deque
        visited = {cell}
        q = deque([cell])
        while q:
            c, r = q.popleft()
            if self.omap.is_free(c, r):
                return (c, r)
            for dc, dr, _ in DIRECTIONS_8:
                nb = (c + dc, r + dr)
                if nb not in visited and self.omap.in_bounds(nb[0], nb[1]):
                    if abs(nb[0] - cell[0]) <= search_r and \
                       abs(nb[1] - cell[1]) <= search_r:
                        visited.add(nb)
                        q.append(nb)
        return None

    # ── Publishers ────────────────────────────────────────────────────────
    def _publish_path(self, cells: list[tuple]):
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for col, row in cells:
            wx, wy = self.omap.g2w(col, row)
            ps = PoseStamped()
            ps.header      = msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _publish_costmap(self):
        msg             = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = self.omap.res
        msg.info.width      = self.omap.w
        msg.info.height     = self.omap.h
        msg.info.origin.position.x = self.omap.ox
        msg.info.origin.position.y = self.omap.oy
        msg.info.origin.orientation.w = 1.0
        flat = (self.omap.grid.flatten() * 100).astype(np.int8)
        msg.data = flat.tolist()
        self.grid_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(AStarPlannerNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

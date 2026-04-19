import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np
import heapq

class DStarLitePlanner(Node):
    def __init__(self):
        super().__init__('planner_global_dstar')
        
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('grid_width', 80)
        self.declare_parameter('grid_height', 80)
        self.declare_parameter('grid_origin_x', -4.0)
        self.declare_parameter('grid_origin_y', -4.0)
        self.declare_parameter('inflation_radius', 0.35) # UPDATED: Tuned value
        self.declare_parameter('occupancy_threshold', 0.5)
        
        self.res = 0.1
        self.width = 200
        self.height = 200
        self.grid = np.zeros((self.width, self.height), dtype=np.int32)
        
        self.origin_x = -10.0
        self.origin_y = -10.0
        self.inflation = 0.22  # TUNED: Optimized for 0.5m gaps in The Gauntlet
        self.obstacle_dist = np.full((self.width, self.height), 10.0) 
        
        self.g = np.full((self.width, self.height), float('inf'))
        self.rhs = np.full((self.width, self.height), float('inf'))
        self.queue = []
        self.in_queue = set()
        self.km = 0.0
        
        self.current_pose = None
        self.pose_history = []
        self.hit_grid = np.zeros((self.width, self.height), dtype=int)
        self.goal_pose = None
        self.start_node = None
        self.goal_node = None
        self.last_node = None
        
        # Correctly subscribe to BOTH sensors and pass mounting offsets
        self.create_subscription(LaserScan, 'front_scan', 
            lambda msg: self.scan_callback(msg, x_offset=0.13, yaw_offset=0.0), 10)
        self.create_subscription(LaserScan, 'rear_scan', 
            lambda msg: self.scan_callback(msg, x_offset=-0.13, yaw_offset=math.pi), 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        
        self.path_pub = self.create_publisher(Path, 'plan', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'local_costmap', 10)
        self.marker_pub = self.create_publisher(Marker, '/grid_markers', 10)
        self.frontier_pub = self.create_publisher(Marker, '/search_frontier', 10)
        
        self.timer = self.create_timer(0.2, self.planning_loop)
        self.viz_timer = self.create_timer(2.0, self.viz_loop)
        self.scrub_timer = self.create_timer(0.1, self.clear_robot_footprint)

        self.pre_populate_static_map()   # mark walls + known interior obstacles
        self.get_logger().info("D* Lite Global Planner Initialized (Static map pre-loaded)")

    def pre_populate_static_map(self):
        """
        Pre-mark known static obstacles in the D* Lite grid.
        Matches the 6×6 patrol arena (walls at ±3 m, T=0.15 m).
        Also marks patrol_arena interior walls — harmless if using
        a different world (LiDAR will correct any wrong marks at runtime).
        """
        INF  = 3.0    # arena half-size
        T    = 0.15   # wall thickness
        INFL = 0.20   # extra inflation for robot clearance

        # ── Outer perimeter walls ─────────────────────────────────────────
        # North / South (full width)
        self.add_static_box(0,      INF,  INF + T/2,      T/2 + INFL)  # N wall
        self.add_static_box(0,     -INF,  INF + T/2,      T/2 + INFL)  # S wall
        # East / West (inner height)
        self.add_static_box( INF,   0,    T/2 + INFL,     INF - T/2)   # E wall
        self.add_static_box(-INF,   0,    T/2 + INFL,     INF - T/2)   # W wall

        # ── patrol_arena interior obstacles ──────────────────────────────
        # NW vertical divider: x=−1.2, centre y=+1.1625, half-len=1.7625
        self.add_static_box(-1.2,  1.1625, 0.06 + INFL, 1.7625 + INFL)
        # p1 stub: y=+1.0, centre x=+1.6625, half-len=1.2625
        self.add_static_box(1.6625,  1.0,  1.2625 + INFL, 0.06 + INFL)
        # p2 stub: y=−0.8, centre x=−1.1625, half-len=1.7625
        self.add_static_box(-1.1625, -0.8, 1.7625 + INFL, 0.06 + INFL)
        # SE pillar: (1.2, −1.6), 0.20×0.20
        self.add_static_box(1.2,  -1.6,   0.10 + INFL, 0.10 + INFL)

        self.get_logger().info(
            "Static map: perimeter walls + patrol_arena interior pre-loaded")

    def add_static_box(self, cx, cy, sx, sy):
        grid_sx = int(sx / self.res / 2) + 2 
        grid_sy = int(sy / self.res / 2) + 2
        gx, gy = self.world_to_grid(cx, cy)
        for dx in range(-grid_sx, grid_sx + 1):
            for dy in range(-grid_sy, grid_sy + 1):
                node = (gx + dx, gy + dy)
                if 0 <= node[0] < self.width and 0 <= node[1] < self.height:
                    self.grid[node] = 1
                    self.hit_grid[node] = 10 
                    self.update_vertex(node)

    def world_to_grid(self, x, y):
        gx = int((x / self.res) + 100)
        gy = int((y / self.res) + 100)
        return (max(0, min(gx, 199)), max(0, min(gy, 199)))
    
    def grid_to_world(self, gx, gy):
        wx = (gx - 100) * self.res
        wy = (gy - 100) * self.res
        return (wx, wy)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.start_node = self.world_to_grid(self.current_pose.position.x, self.current_pose.position.y)
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pose_history.append((now, self.current_pose))
        if len(self.pose_history) > 100: self.pose_history.pop(0)

    def goal_callback(self, msg):
        target = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)
        if target == self.goal_node: return
        self.goal_node = target
        self.g.fill(float('inf'))
        self.rhs.fill(float('inf'))
        self.queue = []
        self.in_queue = set()
        self.rhs[self.goal_node] = 0
        if self.start_node is not None:
            self.last_node = self.start_node
            heapq.heappush(self.queue, (self.calculate_key(self.goal_node), self.goal_node))
            self.in_queue.add(self.goal_node)
            self.compute_shortest_path()
            self.planning_loop()
        else:
            sentinel_key = (0.0, 0.0)
            heapq.heappush(self.queue, (sentinel_key, self.goal_node))
            self.in_queue.add(self.goal_node)

    def scan_callback(self, msg, x_offset, yaw_offset):
        """Map LiDAR points to the global grid while accounting for sensor orientation.
        
        The yaw_offset (0 for front, pi for rear) fixes the mirror effect where
        rear-facing obstacles were being projected into the front of the robot.
        """
        if not self.pose_history: return
        scan_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        best_pose = self.current_pose
        min_diff = float('inf')
        for t, p in reversed(self.pose_history):
            diff = abs(scan_time - t)
            if diff < min_diff:
                min_diff = diff
                best_pose = p
            if diff > 0.1: break 
        self.clear_robot_footprint(best_pose, 0.4)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        changes = []
        yaw = self.get_yaw(best_pose.orientation)
        
        # Sensor mount position in world frame
        wx_start = best_pose.position.x + x_offset * math.cos(yaw)
        wy_start = best_pose.position.y + x_offset * math.sin(yaw)
        start_node = self.world_to_grid(wx_start, wy_start)
        
        unique_hits = set()
        for r, ang in zip(msg.ranges, angles):
            if not (msg.range_min < r < msg.range_max): continue
            if r < 0.25: continue  # Exclude self-detections (bucket walls ~0.15-0.21m)
            
            total_angle = ang + yaw + yaw_offset
            wx_hit = wx_start + r * math.cos(total_angle)
            wy_hit = wy_start + r * math.sin(total_angle)
            hit_node = self.world_to_grid(wx_hit, wy_hit)
            if hit_node: unique_hits.add(hit_node)
        
        # Ray clearing (Aggressive to remove ghosts)
        # We sample every 3rd ray to maintain 10Hz performance while ensuring total coverage
        for i in range(0, len(msg.ranges), 3):
            r = msg.ranges[i]
            if r < msg.range_min: continue
            # If the ray is out of range, we clear up to range_max
            clear_dist = min(r, msg.range_max) if r > msg.range_min else msg.range_max
            
            total_angle = angles[i] + yaw + yaw_offset
            wx_end = wx_start + clear_dist * math.cos(total_angle)
            wy_end = wy_start + clear_dist * math.sin(total_angle)
            end_node = self.world_to_grid(wx_end, wy_end)
            
            ray_cells = self.get_line(start_node, end_node)
            for cell in ray_cells[:-1]: 
                if self.hit_grid[cell] > 0:
                    # Clear faster than we hit
                    self.hit_grid[cell] = max(0, self.hit_grid[cell] - 2)
                    if self.hit_grid[cell] == 0 and self.grid[cell] == 1:
                        self.grid[cell] = 0
                        changes.append(cell)
        
        # Obstacle Hitting & Dilatation
        dilated_hits = set()
        grid_r = int(self.inflation / self.res)
        for hit in unique_hits:
            for dx in range(-grid_r, grid_r + 1):
                for dy in range(-grid_r, grid_r + 1):
                    if math.sqrt(dx**2 + dy**2) <= grid_r:
                        neighbor = (hit[0]+dx, hit[1]+dy)
                        if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                            dilated_hits.add(neighbor)
        
        for node in dilated_hits:
            self.hit_grid[node] = min(10, self.hit_grid[node] + 1)
            if self.hit_grid[node] >= 3 and self.grid[node] == 0:
                self.grid[node] = 1
                changes.append(node)
                
        # Robust Footprint Clearing (Prevent robot from being blocked by its own chassis)
        safe_r = int(0.22 / self.res) + 1  
        for dx in range(-safe_r, safe_r + 1):
            for dy in range(-safe_r, safe_r + 1):
                if math.sqrt(dx**2 + dy**2) <= safe_r:
                    footprint = (start_node[0]+dx, start_node[1]+dy)
                    if 0 <= footprint[0] < self.width and 0 <= footprint[1] < self.height:
                        if self.grid[footprint] == 1:
                            self.grid[footprint] = 0
                            self.hit_grid[footprint] = 0
                            changes.append(footprint)
        
        if changes:
            # Throttle recomputation if node is lagging
            self.recompute_obstacle_dist()  
            affected = set()
            for node in changes:
                affected.add(node)
                for v in self.get_neighbors(node):
                    affected.add(v)
            for node in affected:
                self.update_vertex(node)
            if self.start_node in affected:
                self.compute_shortest_path()

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def calculate_key(self, s):
        h = self.heuristic(self.start_node, s)
        k1 = min(self.g[s], self.rhs[s]) + h + self.km
        k2 = min(self.g[s], self.rhs[s])
        return (k1, k2)

    def heuristic(self, s1, s2):
        return math.sqrt((s1[0]-s2[0])**2 + (s1[1]-s2[1])**2)

    def update_vertex(self, u):
        if u != self.goal_node:
            min_rhs = float('inf')
            for v in self.get_neighbors(u):
                min_rhs = min(min_rhs, self.g[v] + self.cost(u, v))
            self.rhs[u] = min_rhs
        self.in_queue.discard(u)
        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.queue, (self.calculate_key(u), u))
            self.in_queue.add(u)

    def compute_shortest_path(self):
        if not self.start_node or not self.goal_node: return
        iters = 0
        while self.queue and iters < 80000:
            k_old, u = heapq.heappop(self.queue)
            if u not in self.in_queue: continue
            self.in_queue.discard(u)
            k_new = self.calculate_key(u)
            if type(k_old) == type(k_new) and k_old < k_new:
                heapq.heappush(self.queue, (k_new, u))
                self.in_queue.add(u)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for v in self.get_neighbors(u):
                    self.update_vertex(v)
            else:
                self.g[u] = float('inf')
                for v in [u] + self.get_neighbors(u):
                    self.update_vertex(v)
            iters += 1
            if self.queue:
                k_top, k_start = self.queue[0][0], self.calculate_key(self.start_node)
                if type(k_top) == type(k_start) and k_top >= k_start and self.rhs[self.start_node] == self.g[self.start_node]:
                    break

    def get_neighbors(self, u):
        neighbors = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            nx, ny = u[0]+dx, u[1]+dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                neighbors.append((nx, ny))
        return neighbors

    def cost(self, u, v):
        if self.grid[v] == 1: return float('inf')
        base_cost = math.sqrt((u[0]-v[0])**2 + (u[1]-v[1])**2)
        min_d = self.obstacle_dist[v[0], v[1]]
        # Exponential penalty: Drops off fast so we aren't "afraid" of gaps
        penalty = 10.0 * math.exp(-6.0 * min_d)
        return base_cost + penalty

    def recompute_obstacle_dist(self):
        try:
            from scipy.ndimage import distance_transform_edt
            free_mask = (self.grid == 0)
            if not np.any(~free_mask):
                self.obstacle_dist.fill(10.0)
                return
            self.obstacle_dist = distance_transform_edt(free_mask).astype(np.float32)
            np.clip(self.obstacle_dist, 0, 10.0, out=self.obstacle_dist)
        except ImportError:
            obs_coords = np.argwhere(self.grid == 1)
            if len(obs_coords) == 0:
                self.obstacle_dist.fill(10.0)
                return
            if len(obs_coords) > 500:
                idx = np.random.choice(len(obs_coords), 500, replace=False)
                obs_coords = obs_coords[idx]
            all_x, all_y = np.meshgrid(np.arange(self.width), np.arange(self.height), indexing='ij')
            self.obstacle_dist.fill(10.0)
            for ox, oy in obs_coords:
                d = np.sqrt((all_x - ox)**2 + (all_y - oy)**2)
                np.minimum(self.obstacle_dist, d, out=self.obstacle_dist)

    def reset_planner(self):
        """Initialize the D* Lite priority queue with the goal node."""
        self.g.fill(float('inf'))
        self.rhs.fill(float('inf'))
        self.queue = []
        self.in_queue = set()
        self.km = 0.0
        if self.goal_node:
            self.rhs[self.goal_node] = 0
            k = self.calculate_key(self.goal_node)
            heapq.heappush(self.queue, (k, self.goal_node))
            self.in_queue.add(self.goal_node)

    def planning_loop(self):
        if not self.start_node or not self.goal_node: return
        if self.last_node is None:
            self.last_node = self.start_node
            self.recompute_obstacle_dist()
            self.reset_planner()
        if self.start_node != self.last_node:
            self.km += self.heuristic(self.last_node, self.start_node)
            self.last_node = self.start_node
        if self.g[self.start_node] == float('inf'):
            self.compute_shortest_path()
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"
        curr, visited, path_broken = self.start_node, set(), False
        for _ in range(self.width * self.height):
            if curr == self.goal_node: break
            if curr in visited: {path_broken := True}; break
            visited.add(curr)
            best_v, min_g = None, float('inf')
            for v in self.get_neighbors(curr):
                if self.grid[v] == 0 and self.g[v] < min_g and self.g[v] < self.g[curr] - 0.001:
                    min_g, best_v = self.g[v], v
            if best_v is None: {path_broken := True}; break
            curr = best_v
            wx, wy = self.grid_to_world(curr[0], curr[1])
            
            p = PoseStamped()
            p.header.frame_id = "odom"
            p.pose.position.x = wx
            p.pose.position.y = wy
            path_msg.poses.append(p)
        
        if path_broken:
            self.compute_shortest_path()

        # --- SMOOTHING OPTIMIZATION ---
        if len(path_msg.poses) > 5:
            path_msg.poses = self._smooth_path(path_msg.poses)
        
        # Publish when: goal reached (any length path) OR path is long enough to be useful
        reached_goal = (curr == self.goal_node)
        has_useful_path = len(path_msg.poses) >= 2
        
        self.get_logger().info(
            f"D* status: g[start]={self.g[self.start_node]:.1f} "
            f"waypoints={len(path_msg.poses)} reached_goal={reached_goal}",
            throttle_duration_sec=2.0)
        
        if path_msg.poses and (reached_goal or has_useful_path):
            self.path_pub.publish(path_msg)

    def _smooth_path(self, poses):
        """Passes a 3-point moving average (Bezier approximation) to fluidize the path."""
        if len(poses) < 3: return poses
        new_poses = [poses[0]]
        for i in range(1, len(poses) - 1):
            p_prev = poses[i-1].pose.position
            p_curr = poses[i].pose.position
            p_next = poses[i+1].pose.position
            
            # Simple 1-2-1 weighted smoothing
            s = PoseStamped()
            s.header.frame_id = "odom"
            s.pose.position.x = 0.25*p_prev.x + 0.5*p_curr.x + 0.25*p_next.x
            s.pose.position.y = 0.25*p_prev.y + 0.5*p_curr.y + 0.25*p_next.y
            new_poses.append(s)
        new_poses.append(poses[-1])
        return new_poses

    def viz_loop(self):
        self.publish_grid_markers()
        self.publish_frontier_markers()

    def get_line(self, start, end):
        x0, y0, x1, y1 = start[0], start[1], end[0], end[1]
        dx, dy, x, y = abs(x1 - x0), abs(y1 - y0), x0, y0
        sx, sy, cells = (-1 if x0 > x1 else 1), (-1 if y0 > y1 else 1), []
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        cells.append((x, y))
        return cells

    def publish_frontier_markers(self):
        marker = Marker()
        marker.header.frame_id, marker.ns, marker.id, marker.type, marker.action = "odom", "frontier", 1, Marker.CUBE_LIST, Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = self.res*0.8, self.res*0.8, 0.03
        marker.color.a, marker.color.r, marker.color.g, marker.color.b = 0.5, 0.2, 1.0, 0.2
        if not self.start_node: {self.frontier_pub.publish(marker)}; return
        count = 0
        for item in list(self.queue)[:200]:
            node = item[1]
            if node in self.in_queue and math.sqrt((node[0]-self.start_node[0])**2 + (node[1]-self.start_node[1])**2) <= 30:
                wx, wy = self.grid_to_world(node[0], node[1])
                p = Point()
                p.x, p.y, p.z = wx, wy, 0.02
                marker.points.append(p)
                count += 1
                if count >= 100: break
        self.frontier_pub.publish(marker)

    def publish_grid_markers(self):
        marker = Marker()
        marker.header.frame_id, marker.header.stamp, marker.ns, marker.id, marker.type, marker.action = "odom", self.get_clock().now().to_msg(), "occupied_grid", 0, Marker.CUBE_LIST, Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = self.res, self.res, 0.15
        marker.color.a, marker.color.r, marker.color.g, marker.color.b = 0.7, 0.9, 0.1, 0.1
        if not self.start_node: {self.marker_pub.publish(marker)}; return
        rx, ry, view_r = self.start_node[0], self.start_node[1], int(5.0 / self.res)
        for gx in range(max(0, rx-view_r), min(self.width, rx+view_r)):
            for gy in range(max(0, ry-view_r), min(self.height, ry+view_r)):
                if self.grid[gx, gy] == 1 and self.hit_grid[gx, gy] >= 3:
                    wx, wy = self.grid_to_world(gx, gy)
                    p = Point()
                    p.x, p.y, p.z = wx, wy, 0.07
                    marker.points.append(p)
        self.marker_pub.publish(marker)

    def clear_robot_footprint(self, pose=None, radius=0.5):
        p = pose if pose else self.current_pose
        if not p: return
        c = self.world_to_grid(p.position.x, p.position.y)
        gr, changes = int(radius / self.res), []
        for dx in range(-gr, gr + 1):
            for dy in range(-gr, gr + 1):
                if math.sqrt(dx**2 + dy**2) <= gr:
                    node = (c[0]+dx, c[1]+dy)
                    if 0 <= node[0] < self.width and 0 <= node[1] < self.height:
                        if self.grid[node] == 1:
                            self.grid[node] = 0
                            self.hit_grid[node] = 0
                            changes.append(node)
        for node in changes: self.update_vertex(node)

def main(args=None):
    rclpy.init(args=args)
    node = DStarLitePlanner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()

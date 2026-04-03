import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math
import heapq

class NodeAStar:
    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_node')
        
        # Publishers and Subscribers
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.obs_marker_pub = self.create_publisher(MarkerArray, '/map_markers', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.current_pose = None
        self.get_logger().info("A* Global SLAM Planner Started.")
        
        self.reso = 0.1 # This will be updated by the /map message
        self.rr = 0.45  # Increased Inflation (0.45m) to keep robot centered in gaps
        self.obmap = None
        self.min_x, self.min_y = 0.0, 0.0
        self.x_w, self.y_w = 0, 0
        
        # SLAM Map Subscriber
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Timer to keep visualization active
        self.timer = self.create_timer(5.0, self.publish_diagnostics)

    def map_callback(self, msg):
        """Processes the incoming SLAM OccupancyGrid into an A* searchable map."""
        self.reso = msg.info.resolution
        self.x_w = msg.info.width
        self.y_w = msg.info.height
        self.min_x = msg.info.origin.position.x
        self.min_y = msg.info.origin.position.y
        
        # Convert 1D list to 2D Boolean Obstacle Map with Inflation
        data = msg.data
        new_map = [[False for _ in range(self.y_w)] for _ in range(self.x_w)]
        
        # Inflation radius in grid cells
        inflation_cells = int(self.rr / self.reso)
        
        for i in range(len(data)):
            if data[i] > 60: # Obstacle threshold
                ix = i % self.x_w
                iy = i // self.x_w
                # Inflate
                for dx in range(-inflation_cells, inflation_cells + 1):
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        nx, ny = ix + dx, iy + dy
                        if 0 <= nx < self.x_w and 0 <= ny < self.y_w:
                            new_map[nx][ny] = True
        
        self.obmap = new_map
        self.get_logger().info(f"Updated A* Map: {self.x_w}x{self.y_w} (Inflated by {inflation_cells} cells)", once=True)

    def publish_diagnostics(self):
        # We can publish debug markers here if needed
        pass

    def odom_callback(self, msg):
        # NaN Guard
        if math.isnan(msg.pose.pose.position.x): return
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        # NaN Guard
        if math.isnan(msg.pose.position.x): return
        if self.current_pose is None:
            self.get_logger().warn("A* waiting for Odometry...")
            return

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        # Use real robot position as start
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        
        self.get_logger().info(f"A* Planning from [{start_x:.1f}, {start_y:.1f}] to [{goal_x:.1f}, {goal_y:.1f}]")
        
        if self.obmap is None:
            self.get_logger().warn("A* Map is EMPTY! Initializing 'Free Space' search until SLAM publishes...")
            # Create a temporary 40m x 40m empty grid so we can start moving
            self.reso = 0.2
            self.x_w, self.y_w = 200, 200
            self.min_x, self.min_y = -20.0, -20.0
            self.obmap = [[False for _ in range(self.y_w)] for _ in range(self.x_w)]

        # Verify if Goal is valid using cached map
        ngo_idx_x = self.calc_xy_index(goal_x, self.min_x)
        ngo_idx_y = self.calc_xy_index(goal_y, self.min_y)
        ngoal = NodeAStar(ngo_idx_x, ngo_idx_y, 0.0, -1)
        
        if not self.verify_node(ngoal, self.obmap, self.x_w, self.y_w):
            self.get_logger().warn(f"Goal ({goal_x:.2f}, {goal_y:.2f}) is blocked. Finding closest reachable point...")
            # We don't return here anymore, a_star_planning will handle the fallback
            pass

        rx, ry = self.a_star_planning(start_x, start_y, goal_x, goal_y)
        
        if not rx:
            self.get_logger().warn("A* could not find a path!")
            return
            
        # Reverse path to start->goal
        rx.reverse()
        ry.reverse()
        
        # PATH SMOOTHING (Line-of-Sight / Shortcut)
        if len(rx) > 3:
            s_rx, s_ry = [rx[0]], [ry[0]]
            curr = 0
            while curr < len(rx) - 1:
                next_wp = curr + 1
                for test in range(len(rx)-1, curr + 1, -1):
                    if self.is_line_clear(rx[curr], ry[curr], rx[test], ry[test]):
                        next_wp = test
                        break
                s_rx.append(rx[next_wp])
                s_ry.append(ry[next_wp])
                curr = next_wp
            rx, ry = s_rx, s_ry

        # Ensure final path is published
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for ix, iy in zip(rx, ry):
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.pose.position.x = ix
            pose.pose.position.y = iy
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"A* Path Published with {len(rx)} smoothed waypoints.")

    def a_star_planning(self, sx, sy, gx, gy):
        nstart = NodeAStar(self.calc_xy_index(sx, self.min_x), self.calc_xy_index(sy, self.min_y), 0.0, -1)
        ngoal = NodeAStar(self.calc_xy_index(gx, self.min_x), self.calc_xy_index(gy, self.min_y), 0.0, -1)

        # Use cached map to save 5+ seconds
        obmap, x_w, y_w = self.obmap, self.x_w, self.y_w

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart, 0, 0, x_w)] = nstart

        motion = self.get_motion_model()

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o]))
            current = open_set[c_id]

            if current.x == ngoal.x and current.y == ngoal.y:
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break
            
            # FALLBACK: If goal is unreachable, we will eventually empty the open_set.
            # We then find the node in closed_set that was closest to the goal.
            if len(open_set) == 0:
                self.get_logger().warn("Goal Unreachable. Retrieving closest possible point.")
                best_node = None
                min_dist = float('inf')
                for node in closed_set.values():
                    d = self.calc_heuristic(ngoal, node)
                    if d < min_dist:
                        min_dist, best_node = d, node
                if best_node:
                    ngoal = best_node # Pivot goal to the closest reachable point
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for i, _ in enumerate(motion):
                node = NodeAStar(current.x + motion[i][0], current.y + motion[i][1],
                                 current.cost + motion[i][2], c_id)
                n_id = self.calc_grid_index(node, 0, 0, x_w)

                if not self.verify_node(node, obmap, x_w, y_w):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node
        
        # Grid indices to world positions
        rx, ry = self.calc_final_path(ngoal, closed_set)
        return rx, ry

    def calc_final_path(self, ngoal, closed_set):
        rx, ry = [self.calc_grid_position(ngoal.x, self.min_x)], [self.calc_grid_position(ngoal.y, self.min_y)]
        pind = ngoal.pind
        while pind != -1:
            n = closed_set[pind]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            pind = n.pind
        return rx, ry

    def calc_heuristic(self, n1, n2):
        w = 1.0 # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_p):
        pos = index * self.reso + min_p
        return pos

    def calc_xy_index(self, position, min_p):
        return round((position - min_p) / self.reso)

    def calc_grid_index(self, node, min_x, min_y, x_w):
        return (node.y - min_y) * x_w + (node.x - min_x)

    def verify_node(self, node, obmap, x_w, y_w):
        if node.x < 0 or node.y < 0 or node.x >= x_w or node.y >= y_w:
            return False
        if obmap[int(node.x)][int(node.y)]:
            return False
        return True

    def is_line_clear(self, x1, y1, x2, y2):
        """Checks if a straight line between two world points is obstacle-free."""
        idx1_x, idx1_y = self.calc_xy_index(x1, self.min_x), self.calc_xy_index(y1, self.min_y)
        idx2_x, idx2_y = self.calc_xy_index(x2, self.min_x), self.calc_xy_index(y2, self.min_y)
        
        # Supercover Bresenham or simple sampling
        dist = math.hypot(idx2_x - idx1_x, idx2_y - idx1_y)
        if dist < 1.0: return True
        
        steps = int(dist * 2) # Sample at 0.5 grid resolution
        for i in range(steps + 1):
            t = i / steps
            curr_x = int(round(idx1_x + t * (idx2_x - idx1_x)))
            curr_y = int(round(idx1_y + t * (idx2_y - idx1_y)))
            if not self.verify_node(NodeAStar(curr_x, curr_y, 0, 0), self.obmap, self.x_w, self.y_w):
                return False
        return True

    def calc_obstacle_map(self, ox, oy, reso, vr):
        x_w = int((self.max_x - self.min_x) / reso)
        y_w = int((self.max_y - self.min_y) / reso)

        obmap = [[False for _ in range(y_w)] for _ in range(x_w)]
        for ix in range(x_w):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(y_w):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= vr:
                        obmap[ix][iy] = True
                        break
        return obmap, x_w, y_w

    def get_motion_model(self):
        # dx, dy, cost
        return [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
                [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except rclpy._rclpy_pybind11.RCLError:
                pass

if __name__ == '__main__':
    main()

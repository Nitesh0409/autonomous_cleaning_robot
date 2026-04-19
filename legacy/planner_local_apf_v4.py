import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float64
import math
import numpy as np
import csv
import os

class APFPlannerNode(Node):
    """
    Advanced Artificial Potential Field (APF) Navigation Node.
    Generation 4: Differential Geometry & Real Dimensions.
    """
    def __init__(self):
        super().__init__('planner_local_apf')
        
        # Declare Parameters
        self.declare_parameter('k_att', 1.2, ParameterDescriptor(description='Attractive force gain'))
        self.declare_parameter('k_rep', 0.08, ParameterDescriptor(description='Repulsive force gain'))
        self.declare_parameter('k_curl', 0.1, ParameterDescriptor(description='Tangential/Vortex force gain'))
        self.declare_parameter('d0', 1.0, ParameterDescriptor(description='Influence range of obstacles (meters)'))
        self.declare_parameter('v_max', 0.7, ParameterDescriptor(description='Max linear velocity'))
        self.declare_parameter('w_max', 6.0, ParameterDescriptor(description='Max angular velocity'))
        self.declare_parameter('nav_version', 4, ParameterDescriptor(description='1: Legacy, 2: Inflation, 3: Zero-Radius, 4: Diff-Drive'))
        self.declare_parameter('cluster_dist', 0.25, ParameterDescriptor(description='Threshold for clustering LIDAR points'))

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/force_markers', 10)
        self.pub_nav_markers = self.create_publisher(MarkerArray, '/navigation_markers', 10)
        self.pub_obs_markers = self.create_publisher(MarkerArray, '/detected_obstacles', 10)
        self.pub_zone_markers = self.create_publisher(MarkerArray, '/obstacle_zones', 10)
        
        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        # State
        self.current_pose = None
        self.goal_pose = None
        self.waypoints = []
        self.latest_scan = None
        self.goal_reached = False
        self.wheel_positions = {}
        self.start_pose = None
        self.actual_path = Path()
        self.actual_path.header.frame_id = "odom"
        self.trail_pub = self.create_publisher(Path, '/robot_trail', 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # Logging
        self.csv_path = '/mnt/c/SOLIDWORKS Data/browser/Projects/Robotics Assignment/Robot/navigation_log.csv'
        self.init_csv()
        
        self.get_logger().info(f"APF node restored. Gen 4 (Differential) Active.")

    def init_csv(self):
        headers = ['timestamp', 'pos_x', 'pos_y', 'yaw', 'goal_dist', 'v_x', 'v_y', 'w_z']
        with open(self.csv_path, 'w', newline='') as f:
            csv.writer(f).writerow(headers)

    def log_to_csv(self, data):
        with open(self.csv_path, 'a', newline='') as f:
            csv.writer(f).writerow(data)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.start_pose is None: self.start_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.waypoints = []
        self.goal_reached = False

    def path_callback(self, msg):
        self.waypoints = msg.poses
        self.goal_pose = None
        self.goal_reached = False

    def joint_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if 'wheel' in name: self.wheel_positions[name] = pos

    def scan_callback(self, msg):
        self.latest_scan = msg

    def cluster_obstacles(self, msg):
        if not msg: return []
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        cluster_dist = self.get_parameter('cluster_dist').value
        
        clusters = []
        current_cluster = []
        for i in range(len(ranges)):
            r = ranges[i]
            if not (0.1 < r < 5.0):
                if current_cluster: clusters.append(current_cluster); current_cluster = []
                continue
            point = np.array([r * math.cos(angles[i]), r * math.sin(angles[i])])
            if not current_cluster: current_cluster.append(point)
            else:
                if np.linalg.norm(point - current_cluster[-1]) < cluster_dist: current_cluster.append(point)
                else: clusters.append(current_cluster); current_cluster = [point]
        if current_cluster: clusters.append(current_cluster)
        
        obstacles = []
        for c in clusters:
            points = np.array(c)
            centroid = np.mean(points, axis=0)
            min_p, max_p = np.min(points, axis=0), np.max(points, axis=0)
            obstacles.append({
                'center': centroid,
                'radius': np.max(np.linalg.norm(points - centroid, axis=1)),
                'width': max(0.05, max_p[0] - min_p[0]),
                'height': max(0.05, max_p[1] - min_p[1])
            })
        return obstacles

    def control_loop(self):
        if self.current_pose is None: return
        self.publish_visuals()
        
        # 1. Goal Extraction
        target_x, target_y = None, None
        if self.waypoints:
            target_x, target_y = self.waypoints[0].pose.position.x, self.waypoints[0].pose.position.y
        elif self.goal_pose:
            target_x, target_y = self.goal_pose.position.x, self.goal_pose.position.y
        else: return

        if self.goal_reached: self.stop_robot(); return

        # 2. State & Params
        q = self.current_pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y**2 + q.z**2))
        dx_world, dy_world = target_x - self.current_pose.position.x, target_y - self.current_pose.position.y
        dist_to_goal = math.sqrt(dx_world**2 + dy_world**2)

        if dist_to_goal < 0.1:
            if self.waypoints: self.waypoints.pop(0)
            else: self.goal_reached = True; self.stop_robot(); return

        # Live Params
        k_att = self.get_parameter('k_att').value
        k_rep_base = self.get_parameter('k_rep').value
        k_curl = self.get_parameter('k_curl').value
        v_max, w_max = self.get_parameter('v_max').value, self.get_parameter('w_max').value
        nav_version = self.get_parameter('nav_version').value
        
        # 3. Attractive Forces (Body Frame)
        safe_dist = max(0.1, dist_to_goal)
        f_att_world_x, f_att_world_y = k_att * (dx_world / safe_dist), k_att * (dy_world / safe_dist)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        f_att_body_x = f_att_world_x * cos_y + f_att_world_y * sin_y
        f_att_body_y = -f_att_world_x * sin_y + f_att_world_y * cos_y

        # Stagnation Monitor
        if not hasattr(self, '_last_dist'): self._last_dist = dist_to_goal
        self._stuck_ticks = getattr(self, '_stuck_ticks', 0) + 1 if abs(self._last_dist - dist_to_goal) < 0.001 else 0
        self._last_dist = dist_to_goal
        is_trapped = self._stuck_ticks > 40

        # 4. Routing
        self.detected_obstacles = self.cluster_obstacles(self.latest_scan)
        if nav_version == 1:
            msg = self.compute_movement_v1(dist_to_goal, f_att_body_x, f_att_body_y, k_rep_base, k_curl, is_trapped, dy_world, dx_world, yaw, v_max, w_max)
        elif nav_version == 2:
            msg = self.compute_movement_v2(dist_to_goal, f_att_body_x, f_att_body_y, k_rep_base, k_curl, dy_world, dx_world, yaw, v_max, w_max)
        elif nav_version == 3:
            msg = self.compute_movement_v3(dist_to_goal, f_att_body_x, f_att_body_y, k_rep_base, k_curl, dy_world, dx_world, yaw, v_max, w_max)
        else: # Version 4: Differential Geometry
            msg = self.compute_movement_v4(dist_to_goal, f_att_body_x, f_att_body_y, k_rep_base, k_curl, dy_world, dx_world, yaw, v_max, w_max)

        # Telemetry
        now = self.get_clock().now()
        if (now - getattr(self, '_last_log_t', now)).nanoseconds >= 5e8:
            self._last_log_t = now
            self.get_logger().info(f"[TELE] V{nav_version} | Dist:{dist_to_goal:.2f}m | Cmd: vX={msg.linear.x:.2f} wZ={msg.angular.z:.2f}")
            self.log_to_csv([now.nanoseconds/1e9, self.current_pose.position.x, self.current_pose.position.y, yaw, dist_to_goal, msg.linear.x, msg.linear.y, msg.angular.z])

        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def publish_visuals(self):
        if not self.latest_scan: return
        markers = MarkerArray()
        if hasattr(self, 'detected_obstacles'):
            for i, obs in enumerate(self.detected_obstacles):
                # Elevated Transparent Box
                m = Marker()
                m.header.frame_id = "base_footprint"
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "bounding_boxes"; m.id = i
                m.type = Marker.CUBE; m.action = Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = obs['center'][0], obs['center'][1], 0.055
                m.scale.x, m.scale.y, m.scale.z = obs['width'], obs['height'], 0.1
                m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.3)
                markers.markers.append(m)
        self.pub_obs_markers.publish(markers)

        # Influence Zones (d0)
        d0 = self.get_parameter('d0').value
        zone_markers = MarkerArray()
        if hasattr(self, 'detected_obstacles'):
            for i, obs in enumerate(self.detected_obstacles):
                mz = Marker()
                mz.header.frame_id = "base_footprint"
                mz.header.stamp = self.get_clock().now().to_msg()
                mz.ns = "influence_zones"; mz.id = i
                mz.type = Marker.CYLINDER; mz.action = Marker.ADD
                mz.pose.position.x, mz.pose.position.y, mz.pose.position.z = obs['center'][0], obs['center'][1], 0.01
                mz.scale.x, mz.scale.y, mz.scale.z = d0 * 2.0, d0 * 2.0, 0.02
                mz.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.15) # Transparent Orange
                zone_markers.markers.append(mz)
        self.pub_zone_markers.publish(zone_markers)

    def compute_repulsion_v1(self, k_rep, k_curl, is_trapped):
        fx, fy = 0.0, 0.0
        d0 = self.get_parameter('d0').value
        for obs in self.detected_obstacles:
            dist = math.sqrt(obs['center'][0]**2 + obs['center'][1]**2)
            rho = max(0.01, dist - 0.22)
            if rho < d0:
                rep = k_rep * (1.0/rho - 1.0/d0) * (1.0/rho**2)
                ang = math.atan2(obs['center'][1], obs['center'][0])
                fx -= rep * math.cos(ang); fy -= rep * math.sin(ang)
                vortex = k_curl * (2.5 if is_trapped else 1.0) * (2.0 / (rho + 0.1))
                side = 1.0 if obs['center'][1] > 0 else -1.0
                fx += vortex * rep * math.sin(ang) * side
                fy -= vortex * rep * math.cos(ang) * side
        return fx, fy

    def compute_repulsion_v2(self, k_rep):
        fx, fy = 0.0, 0.0
        d0 = self.get_parameter('d0').value
        gain, radius, scaling = k_rep * 40.0, d0, 3.0
        for obs in self.detected_obstacles:
            dist = math.sqrt(obs['center'][0]**2 + obs['center'][1]**2)
            rho = max(0.01, dist - 0.22)
            if rho < radius:
                rep = gain * math.exp(-scaling * (rho / radius))
                ang = math.atan2(obs['center'][1], obs['center'][0])
                fx -= rep * math.cos(ang); fy -= rep * math.sin(ang)
        return fx, fy

    def compute_movement_v1(self, dist_to_goal, f_att_x, f_att_y, k_rep, k_curl, is_trapped, dy, dx, yaw, v_max, w_max):
        fr_x, fr_y = self.compute_repulsion_v1(k_rep, k_curl, is_trapped)
        msg = Twist()
        msg.linear.x = max(min((f_att_x + fr_x) * min(1.0, dist_to_goal/0.5), v_max), -v_max)
        msg.linear.y = max(min((f_att_y + fr_y) * min(1.0, dist_to_goal/0.5), v_max), -v_max)
        err = math.atan2(dy, dx) - yaw
        while err > math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi
        if abs(err) > 0.25: msg.angular.z = max(min(2.0 * err, w_max), -w_max)
        return msg

    def compute_movement_v2(self, dist_to_goal, f_att_x, f_att_y, k_rep, k_curl, dy, dx, yaw, v_max, w_max):
        fr_x, fr_y = self.compute_repulsion_v2(k_rep)
        msg = Twist()
        msg.linear.x = max(min((f_att_x + fr_x) * min(1.0, dist_to_goal/0.5), v_max), -v_max)
        msg.linear.y = max(min((f_att_y + fr_y) * min(1.0, dist_to_goal/0.5), v_max), -v_max)
        return msg

    def compute_movement_v3(self, dist_to_goal, f_att_x, f_att_y, k_rep, k_curl, dy, dx, yaw, v_max, w_max):
        fr_x, fr_y = self.compute_repulsion_v2(k_rep)
        msg = Twist()
        msg.linear.x = max(min((f_att_x + fr_x) * min(1.0, dist_to_goal/0.5), v_max), -v_max)
        msg.linear.y = max(min((f_att_y + fr_y) * min(1.0, dist_to_goal/0.5), v_max), -v_max)
        err = math.atan2(dy, dx) - yaw
        while err > math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi
        if abs(err) > 0.05: msg.angular.z = max(min(4.0 * err, w_max), -w_max)
        elif dist_to_goal < 0.15: msg.angular.z = 2.0
        return msg

    def compute_movement_v4(self, dist_to_goal, f_att_x, f_att_y, k_rep, k_curl, dy, dx, yaw, v_max, w_max):
        fr_x, fr_y = self.compute_repulsion_v2(k_rep)
        f_total_x, f_total_y = f_att_x + fr_x, f_att_y + fr_y
        target_ang = math.atan2(f_total_y, f_total_x)
        msg = Twist()
        if abs(target_ang) > 0.6:
            msg.angular.z = max(min(3.0 * target_ang, w_max), -w_max)
        else:
            mag = math.sqrt(f_total_x**2 + f_total_y**2)
            msg.linear.x = max(min(mag * math.cos(target_ang) * min(1.0, dist_to_goal/0.5), v_max), -v_max)
            msg.angular.z = max(min(2.0 * target_ang, w_max), -w_max)
        msg.linear.y = 0.0
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = APFPlannerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()

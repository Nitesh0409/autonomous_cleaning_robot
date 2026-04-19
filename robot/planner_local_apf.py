import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist, PoseStamped, Point, PoseArray
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float64, Bool
import math
import numpy as np
import csv
import os
import time

class PurePursuitPlannerNode(Node):
    """
    Pure Pursuit + Repulsive Reflex Local Controller.
    Treats Global Path as a 'Magnetic Rail' with safety sidestepping.
    """
    def __init__(self):
        super().__init__('planner_local_apf') # Kept name for launch compatibility
        
        # Pure Pursuit Parameters
        self.lookahead_dist = 0.4  # Optimized for 0.28m chassis
        self.v_max = 0.5
        self.w_max = 1.2           # Increased for sharper pivots
        
        # Artificial Potential Field (Dynamic Obstacle Avoidance)
        self.k_rep = 0.05          # ENABLED: Tuned repulsive gain for dynamic shifting
        self.d0 = 0.8              # Influence radius (80cm)
        self.robot_radius = 0.15
        self.safety_buffer = 0.15  # Increased buffer for dynamic unpredictability
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_nav_markers = self.create_publisher(MarkerArray, 'navigation_markers', 10)
        self.trail_pub = self.create_publisher(Path, 'robot_trail', 10)
        
        # Subscribers
        # Front sensor: mounted at +0.13m, yaw_offset = 0 (facing forward)
        self.create_subscription(LaserScan, '/front_scan',
            lambda msg: self.scan_callback(msg, 0.14, 0.0), 10)
        # Rear sensor: mounted at -0.13m, yaw_offset = pi (facing backward — fixes mirror effect)
        self.create_subscription(LaserScan, '/rear_scan',
            lambda msg: self.scan_callback(msg, -0.14, math.pi), 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, 'plan', self.path_callback, 10)
        self.create_subscription(Bool, '/planner/face_movement', self.face_movement_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(PoseArray, '/perception/semantic_features', self.semantic_features_callback, 10)
        
        # State
        self.current_pose = None
        self.waypoints = []
        self.latest_scan = None
        self.detected_obstacles = []
        self._fused_semantic_memory = []  # From semantic tracker
        self._ema_vx = 0.0
        self._ema_vy = 0.0
        self._alpha = 0.2
        self.face_movement = True  # Default ON
        self.goal_yaw = None        # Orientation from RViz 2D pose arrow
        
        # Logging
        t_str = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(os.getcwd(), f'nav_log_{t_str}.csv')
        self.init_csv()
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"⚓ PURE PURSUIT ACTIVE. Log: {os.path.basename(self.csv_path)}")

    def init_csv(self):
        headers = ['timestamp', 'pos_x', 'pos_y', 'yaw', 'v_x', 'v_y', 'w_z']
        with open(self.csv_path, 'w', newline='') as f:
            csv.writer(f).writerow(headers)

    def log_to_csv(self, data):
        with open(self.csv_path, 'a', newline='') as f:
            csv.writer(f).writerow(data)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.get_logger().info(f"ODOM: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})", throttle_duration_sec=1.0)

    def goal_pose_callback(self, msg):
        """Capture the 2D orientation from RViz SetGoal tool for use in OFF mode."""
        q = msg.pose.orientation
        self.goal_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y**2 + q.z**2))

    def semantic_features_callback(self, msg):
        """Update probabilistic semantic map."""
        self._fused_semantic_memory = [(p.position.x, p.position.y) for p in msg.poses]

    def face_movement_callback(self, msg):
        self.face_movement = msg.data
        mode = 'FACE THE PATH (rotate first)' if self.face_movement else 'FACE 2D GOAL POSE'
        self.get_logger().info(f"Heading Mode: {mode}")

    def path_callback(self, msg):
        self.path = msg
        # Update waypoints FIRST, then run the control loop on fresh data
        if msg.poses:
            self.waypoints = [p.pose.position for p in msg.poses]
            self.get_logger().info(f"🛤️ Rail Updated: {len(self.waypoints)} points.")
        self.get_logger().info(f"🛣️ Path Received: {len(msg.poses)} waypoints. Engaging motors.", throttle_duration_sec=2.0)
        # Force an immediate control cycle
        self.control_loop()

    def scan_callback(self, msg, x_offset, yaw_offset):
        self.latest_scan = msg
        self.cluster_obstacles(msg, x_offset, yaw_offset)

    def cluster_obstacles(self, msg, x_offset, yaw_offset):
        """Project sensor-frame ranges into robot body frame using the full rigid transform.
        
        WITHOUT yaw_offset, a rear sensor reading of cos(0)=forward appears as
        a forward obstacle — the classic mirror effect. Adding yaw_offset corrects this.
        """
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        for i in range(0, len(ranges), 3):
            r = ranges[i]
            if 0.25 < r < 2.0:  # Exclude self-detections (bucket walls at ~0.15m)
                # Full 2D transform: rotate by sensor yaw, then translate by sensor offset
                a = angles[i] + yaw_offset
                x = x_offset + r * math.cos(a)
                y =             r * math.sin(a)
                self.detected_obstacles.append({'x': x, 'y': y, 'r': math.sqrt(x**2 + y**2)})


    def control_loop(self):
        if not self.current_pose or not self.waypoints: 
            # Silence motors when no path is active
            self.cmd_vel_pub.publish(Twist())
            return
        
        rx, ry = self.current_pose.position.x, self.current_pose.position.y
        q = self.current_pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y**2 + q.z**2))
        
        # 0. FIND LOOKAHEAD POINT
        target_pt = self.get_lookahead_point(rx, ry)
        dx_world = target_pt.x - rx
        dy_world = target_pt.y - ry
        dist_to_wp = math.sqrt(dx_world**2 + dy_world**2)
        
        # 1. PROCESS DYNAMIC REFLEX FORCES
        fr_x, fr_y = self.compute_reflex(yaw)
        self.detected_obstacles = []  # Reset for next dual-scan collection
        
        # 2. CALCULATE PURSUIT VECTORS (World Frame)
        dist_to_goal = math.sqrt((self.waypoints[-1].x - rx)**2 + (self.waypoints[-1].y - ry)**2)
        speed = self.v_max * min(1.0, dist_to_goal / 0.8)  # Smoother final stop

        vx_pure = (dx_world / dist_to_wp) * speed if dist_to_wp > 0.01 else 0.0
        vy_pure = (dy_world / dist_to_wp) * speed if dist_to_wp > 0.01 else 0.0

        # 4. CONVERT TO BODY FRAME (Adding Repulsive Shove)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        
        # APF modifies the pure tracking vector before we limit physically
        vx_world_total = vx_pure + fr_x
        vy_world_total = vy_pure + fr_y
        
        # Absolute Dynamic Speed Limiter (Prevent 25m/s flying)
        total_mag = math.sqrt(vx_world_total**2 + vy_world_total**2)
        safe_max = 0.8 # Allow 160% overdrive specifically for fast dynamic evasion
        if total_mag > safe_max:
            vx_world_total = (vx_world_total / total_mag) * safe_max
            vy_world_total = (vy_world_total / total_mag) * safe_max

        vx_body = (vx_world_total) * cos_y + (vy_world_total) * sin_y
        vy_body = -(vx_world_total) * sin_y + (vy_world_total) * cos_y
        
        # 5. SMOOTHING & OUTPUT
        msg = Twist()
        self._ema_vx = (self._alpha * vx_body) + ((1 - self._alpha) * self._ema_vx)
        self._ema_vy = (self._alpha * vy_body) + ((1 - self._alpha) * self._ema_vy)
        
        msg.linear.x = self._ema_vx
        msg.linear.y = self._ema_vy
        
        # 6. HEADING CONTROLLER
        if self.face_movement:
            # Face the Path: rotate FIRST, then move.
            # Target yaw = direction of immediate lookahead (path direction)
            target_yaw = math.atan2(dy_world, dx_world)
            err = target_yaw - yaw
            while err > math.pi: err -= 2*math.pi
            while err < -math.pi: err += 2*math.pi

            # Rotation is always active
            msg.angular.z = max(min(2.0 * err, self.w_max), -self.w_max)

            # Scale linear velocity down by alignment — fully aligned = full speed,
            # facing 90deg wrong = 0 speed, so robot spins in place to align first
            alignment = math.cos(err)              # 1.0 = aligned, 0.0 = 90°, -1.0 = backwards
            drive_scale = max(0.0, alignment)      # Clip to [0, 1] — never reverse due to misalignment
            msg.linear.x *= drive_scale
            msg.linear.y *= drive_scale
        else:
            # OFF: Face the orientation of the RViz 2D pose (the arrow direction)
            # Robot strafes holonomically but gently aligns to the goal arrow yaw
            if self.goal_yaw is not None:
                err = self.goal_yaw - yaw
                while err > math.pi: err -= 2*math.pi
                while err < -math.pi: err += 2*math.pi
                # Gentle rotation toward goal arrow orientation, don't inhibit linear
                msg.angular.z = max(min(1.0 * err, self.w_max), -self.w_max) if abs(err) > 0.08 else 0.0
            else:
                msg.angular.z = 0.0  # No goal set yet — pure strafe

        if dist_to_goal < 0.05:
            if self.waypoints:
                self.get_logger().info("🏁 ARRIVED AT LOCAL GOAL.")
                self.waypoints = []  # Stop
            msg = Twist()

        self.cmd_vel_pub.publish(msg)
        self.publish_markers(target_pt)
        self.log_to_csv([time.time(), rx, ry, yaw, msg.linear.x, msg.linear.y, msg.angular.z])

    def get_lookahead_point(self, rx, ry):
        # Optimized Lookahead: Find point on path closest to target distance
        best_pt = self.waypoints[-1]
        for wp in self.waypoints:
            d = math.sqrt((wp.x - rx)**2 + (wp.y - ry)**2)
            if d > self.lookahead_dist:
                best_pt = wp
                break
        return best_pt

    def compute_reflex(self, yaw):
        fx, fy = 0.0, 0.0
        
        # Fuse semantic map features into the dynamic obstacle arrays automatically 
        for wx, wy in self._fused_semantic_memory:
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            lx = (wx - self.current_pose.position.x) * cos_y + (wy - self.current_pose.position.y) * sin_y
            ly = -(wx - self.current_pose.position.x) * sin_y + (wy - self.current_pose.position.y) * cos_y
            self.detected_obstacles.append({'x': lx, 'y': ly, 'r': math.sqrt(lx**2 + ly**2)})

        for obs in self.detected_obstacles:
            # We treat every cluster point as a small repulsive sphere
            rho = max(0.01, obs['r'] - self.robot_radius - self.safety_buffer)
            if rho < self.d0:
                rep = self.k_rep * (1.0/rho - 1.0/self.d0) * (1.0/rho**2)
                # Convert obstacle local x,y back to world x,y for force sum
                cos_y, sin_y = math.cos(yaw), math.sin(yaw)
                ox_world = obs['x'] * cos_y - obs['y'] * sin_y
                oy_world = obs['x'] * sin_y + obs['y'] * cos_y
                
                fx -= rep * (ox_world / obs['r'])
                fy -= rep * (oy_world / obs['r'])
        
        # Saturate pure reflex vector (before blending with pursuit)
        mag = math.sqrt(fx**2 + fy**2)
        if mag > 1.5:
            fx = (fx / mag) * 1.5
            fy = (fy / mag) * 1.5
        return fx, fy

    def publish_markers(self, target_pt):
        markers = MarkerArray()
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "lookahead"; m.id = 0
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = target_pt.x, target_pt.y, 0.2
        m.scale.x, m.scale.y, m.scale.z = 0.2, 0.2, 0.2
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8) # Yellow Lookahead
        markers.markers.append(m)
        self.pub_nav_markers.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitPlannerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()

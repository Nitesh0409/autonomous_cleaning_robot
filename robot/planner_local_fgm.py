import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import ColorRGBA
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import numpy as np
import math

class FGMPlannerNode(Node):
    def __init__(self):
        super().__init__('planner_local_fgm')
        
        # 1. PARAMETER REGISTRY (Standardized with Instructional Descriptions)
        self.declare_parameter('max_vel', 0.8, ParameterDescriptor(
            description='Top speed (m/s). Increase for fast open runs, decrease for precise navigation.'))
        
        self.declare_parameter('safety_bubble', 0.35, ParameterDescriptor(
            description='Collision radius (m). Increase if robot clips objects, decrease for narrow doorways.'))
        
        self.declare_parameter('slow_zone', 0.55, ParameterDescriptor(
            description='Deceleration trigger (m). Increase if robot overshoots waypoints or stops too late.'))
        
        self.declare_parameter('repulsion_gain', 0.7, ParameterDescriptor(
            description='Magnetic Force. Increase if robot hugs walls too closely or gets stuck in corners.'))
        
        self.declare_parameter('repulsion_dist', 0.75, ParameterDescriptor(
            description='Magnetic Range (m). Increase to start avoiding walls earlier in wide environments.'))
        
        self.declare_parameter('ema_alpha', 0.15, ParameterDescriptor(
            description='Steering Smoothness. Lower (0.05) for elegant curves, higher (0.4) for sharp avoidance.'))

        # SPECIAL BUTTONS (Boolean Triggers)
        self.declare_parameter('reset_to_defaults', False, ParameterDescriptor(
            description='Check this to RESET all sliders to factory tactical defaults.'))
        
        self.declare_parameter('save_to_log', False, ParameterDescriptor(
            description='Check this to LOG current settings to terminal (Copy for permanent use).'))

        # 2. State & Filtering
        self.odom = None
        self.latest_scan = None
        self.global_path = []
        self.current_wp_idx = 0
        self.maneuver_state = "IDLE"
        self.start_time = None
        self.last_steering_angle = 0.0
        self.current_bubble = 0.35 # Internal tracking for adaptive expansion
        self.last_steer_goal = 0.0 # For EMA
        self.gap_lock_angle = 0.0 # For Hysteresis
        self.prev_wp_pos = None # For Projected Advancement
        self.prev_cmd_vx = 0.0 # For Acceleration Limit
        self.prev_cmd_vy = 0.0 # For Acceleration Limit
        
        # Wall Sliding Persistence
        self.drift_pref = 0.0 # 0 for none, 1 for left, -1 for right
        self.escape_latch_time = 0.0
        self.recovery_end_time = 0.0
        self._esc_ang = 0.0
        self.last_drift_time = 0.0
        
        # 3. Visualization State
        self.start_pose = None
        self.actual_path = Path()
        self.actual_path.header.frame_id = "odom"
        
        # 4. Dynamic Parameter Callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # 4. ROS Sub/Pub
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_path = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_trail = self.create_publisher(Path, '/robot_trail', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/navigation_markers', 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.escape_latch_time = 0.0
        self.recovery_end_time = 0.0
        self.get_logger().info("FGM Local Planner (Gap-Flow Stability) Started.")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'reset_to_defaults' and param.value is True:
                self.get_logger().warn("RESET TRIGGERED: Returning to Factory Tactical Defaults.")
                # Non-blocking reset via timer to avoid callback recursion
                self.create_timer(0.1, self.perform_reset, oneshot=True)
            
            if param.name == 'save_to_log' and param.value is True:
                self.log_current_params()
                self.create_timer(0.1, self.clear_save_trigger, oneshot=True)
                
        return SetParametersResult(successful=True)

    def perform_reset(self):
        # SNAP settings back
        self.set_parameters([
            rclpy.parameter.Parameter('max_vel', rclpy.Parameter.Type.DOUBLE, 0.8),
            rclpy.parameter.Parameter('safety_bubble', rclpy.Parameter.Type.DOUBLE, 0.35),
            rclpy.parameter.Parameter('slow_zone', rclpy.Parameter.Type.DOUBLE, 0.40),
            rclpy.parameter.Parameter('repulsion_gain', rclpy.Parameter.Type.DOUBLE, 1.2),
            rclpy.parameter.Parameter('repulsion_dist', rclpy.Parameter.Type.DOUBLE, 1.2),
            rclpy.parameter.Parameter('ema_alpha', rclpy.Parameter.Type.DOUBLE, 0.15),
            rclpy.parameter.Parameter('reset_to_defaults', rclpy.Parameter.Type.BOOL, False)
        ])

    def log_current_params(self):
        self.get_logger().info("--- CURRENT TACTICAL SETTINGS (Save to your launch file) ---")
        self.get_logger().info(f"max_vel: {self.get_parameter('max_vel').value}")
        self.get_logger().info(f"safety: {self.get_parameter('safety_bubble').value}")
        self.get_logger().info(f"gain: {self.get_parameter('repulsion_gain').value}")
        self.get_logger().info(f"smooth: {self.get_parameter('ema_alpha').value}")
        self.get_logger().info("---------------------------------------------------------")

    def clear_save_trigger(self):
        self.set_parameters([rclpy.parameter.Parameter('save_to_log', rclpy.Parameter.Type.BOOL, False)])

    def odom_callback(self, msg): 
        self.odom = msg
        if self.start_pose is None:
            self.start_pose = msg.pose.pose
    def scan_callback(self, msg): self.latest_scan = msg
    def path_callback(self, msg):
        if not msg.poses: return
        self.global_path = msg.poses
        self.current_wp_idx = 0
        self.prev_wp_pos = None # Hard Reset: Forget old progress
        self._l_path_t = 0      # Reset warning timer

    def get_arc_scan_info(self, min_a, max_a):
        """Helper to get min distance within a specific angular arc."""
        if not self.latest_scan: return 10.0, 0.0
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(self.latest_scan.angle_min, self.latest_scan.angle_max, len(ranges))
        mask = (angles >= min_a) & (angles <= max_a)
        filtered = ranges[mask]
        filtered = filtered[np.isfinite(filtered)]
        if len(filtered) == 0: return 10.0, 0.0
        idx = np.argmin(filtered)
        return filtered[idx], angles[mask][idx]

    def get_scan_info(self, min_a, max_a):
        """Original wide scan helper (weighted center)."""
        if not self.latest_scan: return 10.0, 0.0
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(self.latest_scan.angle_min, self.latest_scan.angle_max, len(ranges))
        mask = (angles >= min_a) & (angles <= max_a)
        
        # Safety: Empty scanner window
        if not np.any(mask): 
            return 10.0, (min_a + max_a)/2.0
            
        r_slice = ranges[mask]
        a_slice = angles[mask]
        
        filtered = np.where(np.isfinite(r_slice), r_slice, 10.0)
        weights = 1.0 / (1.0 + 0.5 * np.abs(a_slice - (min_a + max_a)/2.0))
        valid = filtered * weights
        
        if len(valid) == 0:
            return 10.0, (min_a + max_a)/2.0
            
        idx = np.argmin(valid)
        return filtered[idx], a_slice[idx]

    def control_loop(self):
        if not self.odom or not self.latest_scan: return
        
        # 1. State Extraction
        pose = self.odom.pose.pose
        siny_cosp = 2 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y)
        cosy_cosp = 1 - 2 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # 2. IMMEDIATE VISUALS: Topics will now appear in RViz instantly
        self.publish_visuals(pose)
        
        if not self.global_path:
            # Periodic warning to tell user a goal is needed
            if not hasattr(self, '_l_path_t'): self._l_path_t = 0
            self._l_path_t += 1
            if self._l_path_t % 100 == 0:
                self.get_logger().warn("WAITING FOR PATH: Set a goal in RViz (2D Goal Pose)...")
            return
        
        wp = self.global_path[self.current_wp_idx].pose.position
        dist_to_wp = math.sqrt((wp.x - pose.position.x)**2 + (wp.y - pose.position.y)**2)
        
        # WAYPOINT ADVANCEMENT (Dual-Strategy: Distance + Projection)
        # Use previous waypoint to calculate 'Progress' along the path segment
        if self.prev_wp_pos is None: self.prev_wp_pos = pose.position
        
        # Segment Vector
        seg_x, seg_y = wp.x - self.prev_wp_pos.x, wp.y - self.prev_wp_pos.y
        seg_len = math.sqrt(seg_x**2 + seg_y**2)
        
        # Robot Progress Vector (from Start of segment)
        rob_x, rob_y = pose.position.x - self.prev_wp_pos.x, pose.position.y - self.prev_wp_pos.y
        
        # Project Robot onto Segment
        if seg_len > 0.01:
            projection = (rob_x * seg_x + rob_y * seg_y) / seg_len
        else:
            projection = 0.0
            
        # Advance if we are within 0.3m OR if we have projected PAST the waypoint
        if dist_to_wp < 0.3 or projection > seg_len:
            if self.current_wp_idx < len(self.global_path) - 1:
                self.prev_wp_pos = wp # Update segment start
                self.current_wp_idx += 1
                self.get_logger().info(f"Waypoint {self.current_wp_idx} Cleared (Dist: {dist_to_wp:.2f}m, Proj: {projection:.2f}m)")
            else:
                self.maneuver_state = "ARRIVED"
                self.pub_vel.publish(Twist())
                return

        goal_a_world = math.atan2(wp.y - pose.position.y, wp.x - pose.position.x)
        goal_a_body = math.atan2(math.sin(goal_a_world - yaw), math.cos(goal_a_world - yaw))

        # LOAD PARAMS
        max_vel = self.get_parameter('max_vel').value
        target_bubble = self.get_parameter('safety_bubble').value
        
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(self.latest_scan.angle_min, self.latest_scan.angle_max, len(ranges))
        now_sec = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        
        # 1. SCANNING ARCS
        # FRONT ARCS only (Panic Check): -0.7 to +0.7 rad (~40 deg)
        # SIDES are treated as navigation repulsion, not panic.
        dist_panic, ang_panic = self.get_arc_scan_info(-0.7, 0.7)
        dist_direct, _ = self.get_scan_info(goal_a_body - 0.2, goal_a_body + 0.2)
        
        # 2. REPULSION FIELD (Normal + Tangential Sliding)
        # Use a slightly wider arc for general repulsion steering
        rep_x, rep_y = 0.0, 0.0
        rot_scaling = 1.0 # Default speed multiplier
        for i in range(len(ranges)):
            if np.isfinite(ranges[i]) and ranges[i] < 0.6:
                ang = angles[i]
                # Force scales exponentially, but we add a 0.06m floor to stop infinities
                force = 0.25 * (0.45 / (max(0.06, ranges[i])))**2.2
                
                # NORMAL FORCE (Push Away)
                rep_x -= force * math.cos(ang)
                rep_y -= force * math.sin(ang)
                
                # TANGENTIAL FORCE (Slide Along)
                # Rotate normal by 90 deg to get tangent
                tx = -math.sin(ang)
                ty = math.cos(ang)
                
                # DRIFT LATCH: Decide direction if not already locked
                if self.drift_pref == 0:
                    dot_goal = tx * math.cos(goal_a_body) + ty * math.sin(goal_a_body)
                    self.drift_pref = 1.0 if dot_goal > 0 else -1.0
                
                # Apply TANGENT component (Aggressive sliding)
                rep_x += 1.5 * force * tx * self.drift_pref
                rep_y += 1.5 * force * ty * self.drift_pref
                
                # Update cooldown timer
                self.last_drift_time = now_sec

        # TOTAL REPULSION MAGNITUDE CAP: Prevent 400.0m/s launch force!
        # Limit total vector to 1.2m/s (enough to overcome max_vel)
        rep_mag = math.sqrt(rep_x**2 + rep_y**2)
        if rep_mag > 1.2:
            rep_x = (rep_x / rep_mag) * 1.2
            rep_y = (rep_y / rep_mag) * 1.2

        # Reset Drift Latch if no repulsion for 0.8 seconds
        if now_sec - self.last_drift_time > 0.8:
            self.drift_pref = 0.0
        
        # 3. MOTION STATE MACHINE (Directional, Latch-Lock & Recovery)
        steer_goal = goal_a_body
        
        # PANIC: Only check Front Arc, and only if NOT in recovery window
        is_panicking = (dist_panic < self.current_bubble) and (now_sec > self.recovery_end_time)
        is_latching = (now_sec < self.escape_latch_time)

        cmd = Twist()
        if is_panicking or is_latching:
            # LATCH LOCK: Only set the timer if we are starting a NEW escape session
            if not is_latching:
                self.escape_latch_time = now_sec + 1.4 # Solid 1.4s commitment
                # Capture the panic angle to ensure we reverse AWAY from it even if we turn
                self._esc_ang = ang_panic
                self.get_logger().warn(f"LOCKED ESCAPE! Clearance: {dist_panic:.2f}m")
            
            self.maneuver_state = "ESCAPE"
            # MECANUM SIDESTEP: Push AWAY from panic angle with zero rotation
            # This is the 'True Mecanum' fix for the car-like struggle.
            cmd.linear.x = -0.4 * max_vel * math.cos(self._esc_ang)
            cmd.linear.y = -0.8 * max_vel * math.sin(self._esc_ang)
            cmd.angular.z = 0.0 # ROTATION LOCK: Stay steady while escaping
            
            # Record when recovery should end (0.6s after escape finishes)
            self.recovery_end_time = self.escape_latch_time + 0.6
        else:
            # F1TENTH FOLLOW-THE-GAP (FGM)
            # 1. Inflation: Find closest point and create a safety bubble
            closest_idx = np.nanargmin(ranges)
            closest_dist = ranges[closest_idx]
            
            # Virtual Bubble: Inflate the obstacle in the LiDAR data
            # Bubble size is 0.45m to ensure the 0.3m robot clears it
            bubble_radius = 0.45
            angle_res = (self.latest_scan.angle_max - self.latest_scan.angle_min) / len(ranges)
            bubble_width = int(math.atan2(bubble_radius, max(0.1, closest_dist)) / angle_res)
            
            fgm_ranges = np.copy(ranges)
            start_idx = max(0, closest_idx - bubble_width)
            end_idx = min(len(ranges), closest_idx + bubble_width)
            fgm_ranges[start_idx:end_idx] = 0.0
            
            # 2. Find All Gaps in the FORWARD 180 FIELD ONLY
            # This prevents the robot from 'retreating' to a wider gap behind it.
            is_forward = (angles > -1.6) & (angles < 1.6)
            is_safe = (fgm_ranges > 0.6) & is_forward
            all_gaps = []
            curr_start = -1
            for i in range(len(is_safe)):
                if is_safe[i]:
                    if curr_start == -1: curr_start = i
                else:
                    if curr_start != -1:
                        all_gaps.append((curr_start, i-1))
                        curr_start = -1
            if curr_start != -1: all_gaps.append((curr_start, len(is_safe)-1))
            
            # 3. Target Selection: Pick the gap center closest to goal_a_body
            if all_gaps:
                best_gap = all_gaps[0]
                min_diff = 1e9
                for gap in all_gaps:
                    center_idx = (gap[0] + gap[1]) // 2
                    diff = abs(angles[center_idx] - goal_a_body)
                    if diff < min_diff:
                        min_diff = diff
                        best_gap = gap
                
                gap_center_idx = (best_gap[0] + best_gap[1]) // 2
                steer_goal = angles[gap_center_idx]
            else:
                # Emergency: If no safe gaps found, just use repulsion steering
                steer_goal = goal_a_body

            # 4. Goal-Centric Priority (If direct path is safe, take it!)
            goal_idx = int((goal_a_body - self.latest_scan.angle_min) / angle_res)
            goal_idx = np.clip(goal_idx, 0, len(is_safe)-1)
            if is_safe[goal_idx]:
                # If target is safe, stay within 20% of the gap center to keep buffer
                steer_goal = 0.8 * goal_a_body + 0.2 * steer_goal
            
            # 5. DYNAMIC SPEED SCALING: Slow down for tight turns!
            # If turning more than 40 degrees, drop speed to 0.35
            if abs(steer_goal) > 0.7:
                max_vel = 0.35

            self.gap_lock_angle = steer_goal
            
            self.maneuver_state = "GLIDE" if dist_direct > 1.2 else "SQUEEZE"
            
            # ADAPTIVE BUBBLE (Squeeze logic)
            if self.maneuver_state == "SQUEEZE":
                self.current_bubble = max(0.18, self.current_bubble - 0.05)
            else:
                self.current_bubble = min(target_bubble, self.current_bubble + 0.01)
                
            # EMA SMOOTHING (Steering Stability)
            alpha = self.get_parameter('ema_alpha').value
            steer_goal = (1.0 - alpha) * self.last_steer_goal + alpha * steer_goal
            self.last_steer_goal = steer_goal

            # PURE HOLONOMIC FLOW: No speed penalty for turning!
            rot_scaling = 1.0 
            
            # Force Clamping: Ensure repulsion doesn't explode physics
            rep_mag_total = math.sqrt(rep_x**2 + rep_y**2)
            if rep_mag_total > 1.2 * max_vel:
                rep_x = (rep_x / rep_mag_total) * 1.2 * max_vel
                rep_y = (rep_y / rep_mag_total) * 1.2 * max_vel

            # Vector Summary: Combine Goal Steering + Repulsion Forces
            cmd.linear.x = (max_vel * math.cos(steer_goal)) + rep_x
            cmd.linear.y = (max_vel * math.sin(steer_goal)) + rep_y
            
            # PIVOT GATING: If we need a massive turn (>60 deg), stop moving and align first.
            # This prevents the 'spinning joyride' drift.
            if abs(steer_goal) > 1.05:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
            
            # HARD VELOCITY CLAMP
            cmd.linear.x = np.clip(cmd.linear.x, -1.2 * max_vel, 1.2 * max_vel)
            cmd.linear.y = np.clip(cmd.linear.y, -1.2 * max_vel, 1.2 * max_vel)
            
            # ACCELERATION (Slew-Rate) LIMIT: 0.15m/s per 50ms (~3.0 m/s^2)
            max_step = 0.15 
            cmd.linear.x = np.clip(cmd.linear.x, self.prev_cmd_vx - max_step, self.prev_cmd_vx + max_step)
            cmd.linear.y = np.clip(cmd.linear.y, self.prev_cmd_vy - max_step, self.prev_cmd_vy + max_step)
            
            self.prev_cmd_vx, self.prev_cmd_vy = cmd.linear.x, cmd.linear.y
            
            # Spin Damping during Wall-Sliding:
            # If the robot is fighting a wall (Repulsion > 0.3), kill rotation
            rot_damping = 1.0
            rep_total = math.sqrt(rep_x**2 + rep_y**2)
            if rep_total > 0.3:
                rot_damping = 0.2 # 80% rotation reduction
                
            # Small Space Rotation Lock
            # If we are within 0.2m of ANY obstacle, refuse to spin heads-first into it
            min_dist = np.nanmin(ranges)
            if min_dist < 0.22:
                cmd.angular.z = 0.0
            else:
                cmd.angular.z = np.clip(1.2 * (steer_goal) * rot_damping, -1.0, 1.0)
            
            self.last_steering_angle = steer_goal


        self.pub_vel.publish(cmd)
        
        # 5. DEBUG TELEMETRY (Periodic)
        if not hasattr(self, '_log_tick'): self._log_tick = 0
        self._log_tick += 1
        if self._log_tick % 10 == 0: # Log at ~2Hz
            self.get_logger().info(
                f"\n[TACTICAL TRACE]"
                f"\n- Pos: ({pose.position.x:.2f}, {pose.position.y:.2f}) | Yaw: {math.degrees(yaw):.1f}°"
                f"\n- Waypoint: {self.current_wp_idx} | Dist: {dist_to_wp:.2f}m"
                f"\n- Mode: {self.maneuver_state} | Steer: {math.degrees(self.last_steering_angle):.1f}°"
                f"\n- Velocity: x={cmd.linear.x:.2f}, y={cmd.linear.y:.2f}, w={cmd.angular.z:.2f}"
                f"\n- Repulsion: ({rep_x:.2f}, {rep_y:.2f}) | SpeedMod: {rot_scaling:.2f}"
                f"\n" + "-"*30
            )

    def publish_visuals(self, pose):
        # 1. Update Actual Path (Breadcrumbs)
        # Limit update rate to ~5Hz for the path to save bandwidth
        if not hasattr(self, '_path_tick'): self._path_tick = 0
        self._path_tick += 1
        
        if self._path_tick % 4 == 0:
            p_stamped = PoseStamped()
            p_stamped.header.frame_id = "odom"
            p_stamped.header.stamp = self.get_clock().now().to_msg()
            p_stamped.pose = pose
            self.actual_path.poses.append(p_stamped)
            
            # Keep trail history to 250 points (~50 seconds of history)
            if len(self.actual_path.poses) > 250:
                self.actual_path.poses.pop(0)
                
            self.pub_trail.publish(self.actual_path)
        
        # 2. Publish Reference Markers (Start & Origin)
        if self.start_pose:
            m_array = MarkerArray()
            
            # Start Marker (Green Disc)
            start_m = Marker()
            start_m.header.frame_id = "odom"
            start_m.header.stamp = self.get_clock().now().to_msg()
            start_m.ns = "reference"
            start_m.id = 0
            start_m.type = Marker.CYLINDER
            start_m.action = Marker.ADD
            start_m.pose = self.start_pose
            start_m.pose.position.z = 0.005 # Flush to ground
            start_m.scale.x, start_m.scale.y, start_m.scale.z = 0.4, 0.4, 0.01
            start_m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.6)
            m_array.markers.append(start_m)
            
            # Origin Marker (Red Disc)
            orig_m = Marker()
            orig_m.header.frame_id = "odom"
            orig_m.header.stamp = self.get_clock().now().to_msg()
            orig_m.ns = "reference"
            orig_m.id = 1
            orig_m.type = Marker.CYLINDER
            orig_m.action = Marker.ADD
            orig_m.pose.position.x, orig_m.pose.position.y, orig_m.pose.position.z = 0.0, 0.0, 0.0
            orig_m.scale.x, orig_m.scale.y, orig_m.scale.z = 0.5, 0.5, 0.01
            orig_m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
            m_array.markers.append(orig_m)
            
            self.pub_markers.publish(m_array)

def main():
    rclpy.init()
    node = FGMPlannerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__': main()

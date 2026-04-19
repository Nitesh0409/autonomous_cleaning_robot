import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
import math
import numpy as np
import time

class DWAPlannerNode(Node):
    def __init__(self):
        super().__init__('planner_local_dwa')
        
        # 1. PARAMETERS (Auto-Tuned for High Performance)
        self.declare_parameter('max_speed', 0.8)
        self.declare_parameter('max_accel', 3.0)
        self.declare_parameter('predict_time', 2.0)
        self.declare_parameter('dt', 0.1)
        
        self.v_max = self.get_parameter('max_speed').value
        self.a_max = self.get_parameter('max_accel').value
        self.predict_time = self.get_parameter('predict_time').value
        self.dt = self.get_parameter('dt').value
        
        # 2. STATE
        self.current_pose = None
        self.waypoints = []
        self.last_vx, self.last_vy, self.last_wz = 0.0, 0.0, 0.0
        self.scan_points_front = []
        self.scan_points_rear = []
        self.scan_points = []  # merged
        self.face_movement = True
        
        # 3. PUB/SUB
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, 'plan', self.path_callback, 10)
        self.create_subscription(Bool, '/planner/face_movement', self.face_movement_callback, 10)
        # Front sensor: x_offset=+0.13m, yaw_offset=0 (facing forward)
        self.create_subscription(LaserScan, '/front_scan',
            lambda msg: self.scan_callback(msg, 0.13, 0.0, 'front'), 10)
        # Rear sensor: x_offset=-0.13m, yaw_offset=pi (facing backward)
        self.create_subscription(LaserScan, '/rear_scan',
            lambda msg: self.scan_callback(msg, -0.13, math.pi, 'rear'), 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🌊 NITRO-DWA ONLINE (TUNED).")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def face_movement_callback(self, msg):
        self.face_movement = msg.data
        self.get_logger().info(f"Heading Mode: {'ALIGN TO PATH' if self.face_movement else 'HOLONOMIC'}")


    def path_callback(self, msg):
        self.waypoints = [p.pose.position for p in msg.poses]

    def scan_callback(self, msg, x_offset, yaw_offset, sensor_id):
        """Convert sensor-frame ranges to robot body-frame (x,y) points.
        
        Applies the full 2D rigid transform:  
            x_robot = x_offset + r*cos(alpha + yaw_offset)  
            y_robot =             r*sin(alpha + yaw_offset)  
        This prevents the mirror effect caused by ignoring the rear sensor's 180° flip.
        """
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if r > 0.25 and r < msg.range_max:  # 0.25m excludes bucket self-detections
                a = angle + yaw_offset          # apply mounting rotation
                x = x_offset + r * math.cos(a) # apply mounting translation
                y =            r * math.sin(a)
                points.append((x, y))
            angle += msg.angle_increment

        if sensor_id == 'front':
            self.scan_points_front = points
        else:
            self.scan_points_rear = points
        # Merge both sensors so the obstacle list is always complete
        self.scan_points = self.scan_points_front + self.scan_points_rear

    def predict_trajectory(self, vx, vy, wz, rx, ry, yaw):
        traj = []
        curr_x, curr_y, curr_yaw = rx, ry, yaw
        for _ in range(int(self.predict_time / self.dt)):
            dx = (vx * math.cos(curr_yaw) - vy * math.sin(curr_yaw)) * self.dt
            dy = (vx * math.sin(curr_yaw) + vy * math.cos(curr_yaw)) * self.dt
            curr_x += dx
            curr_y += dy
            curr_yaw += wz * self.dt
            traj.append((curr_x, curr_y))
        return traj

    def control_loop(self):
        if not self.current_pose or not self.waypoints: 
            # Send zero velocity while idle
            self.cmd_vel_pub.publish(Twist())
            return
        
        rx, ry = self.current_pose.position.x, self.current_pose.position.y
        q = self.current_pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y**2 + q.z**2))
        
        if not self.waypoints:
            # IDLE MODE: Send zero velocity to ensure robot stays stationary
            self.cmd_vel_pub.publish(Twist())
            return

        lookahead_idx = min(15, len(self.waypoints) - 1)
        target = self.waypoints[lookahead_idx]
        final_goal = self.waypoints[-1]
        
        best_v = (0.0, 0.0, 0.0)
        best_score = -float('inf')
        best_traj = []
        
        V_X_RANGE = np.linspace(-self.v_max, self.v_max, 7)
        V_Y_RANGE = np.linspace(-self.v_max, self.v_max, 7)
        W_Z_RANGE = np.linspace(-1.0, 1.0, 5)
        
        for vx in V_X_RANGE:
            for vy in V_Y_RANGE:
                for wz in W_Z_RANGE:
                    traj = self.predict_trajectory(vx, vy, wz, rx, ry, yaw)
                    if not traj: continue
                    
                    is_colliding = False
                    # Auto-Tuned Trajectory Sampling (9 points)
                    for tx, ty in [traj[0], traj[len(traj)//9], traj[2*len(traj)//9], traj[3*len(traj)//9], traj[4*len(traj)//9], traj[5*len(traj)//9], traj[6*len(traj)//9], traj[7*len(traj)//9], traj[-1]]: 
                        lx = (tx - rx) * math.cos(yaw) + (ty - ry) * math.sin(yaw)
                        ly = -(tx - rx) * math.sin(yaw) + (ty - ry) * math.cos(yaw)
                        
                        for ox, oy in self.scan_points:
                            if (lx - ox)**2 + (ly - oy)**2 < 0.04: # Tuned Collision Radius (0.20m^2)
                                is_colliding = True
                                break
                        if is_colliding: break
                    
                    if is_colliding:
                        score = -1000.0
                    else:
                        end_px, end_py = traj[-1]
                        dist_to_target = math.sqrt((end_px - target.x)**2 + (end_py - target.y)**2)
                        
                        target_yaw = math.atan2(final_goal.y - ry, final_goal.x - rx)
                        yaw_error = target_yaw - (yaw + wz * self.predict_time)
                        while yaw_error > math.pi: yaw_error -= 2*math.pi
                        while yaw_error < -math.pi: yaw_error += 2*math.pi
                        
                        # CLEAN POTENTIAL FIELD SCORE
                        # We only care about: 1. Distance to goal, 2. Alignment with path
                        if self.face_movement:
                            score = -(dist_to_target * 50.0) - (abs(yaw_error) * 5.0)
                        else:
                            # Independent Orientation (holonomic strafe prioritized)
                            # We don't penalize yaw_error as much, or at all, allowing it to naturally rotate 
                            # into whatever yaw avoids collisions best without forcing it to "look" at the goal
                            score = -(dist_to_target * 50.0) - (abs(wz) * 1.5)
                    
                    if score > best_score:
                        best_score = score
                        best_v = (vx, vy, wz)
                        best_traj = traj
        
        # Fan Viz
        marker_array = MarkerArray()
        marker_clear = Marker()
        marker_clear.action = Marker.DELETEALL
        marker_array.markers.append(marker_clear)
        if best_v != (0.0, 0.0, 0.0):
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "best_traj"
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.04
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.4, 1.0, 1.0
            for tx, ty in best_traj:
                p = Point(); p.x, p.y, p.z = tx, ty, 0.05
                m.points.append(p)
            marker_array.markers.append(m)
        self.viz_pub.publish(marker_array)

        # Smooth Motion
        max_dv = self.a_max * 0.1
        target_vx, target_vy, target_wz = best_v
        dvx = max(min(target_vx - self.last_vx, max_dv), -max_dv)
        dvy = max(min(target_vy - self.last_vy, max_dv), -max_dv)
        dwz = max(min(target_wz - self.last_wz, 0.05), -0.05)
        
        self.last_vx += dvx
        self.last_vy += dvy
        self.last_wz += dwz
        
        msg = Twist()
        msg.linear.x = float(self.last_vx)
        msg.linear.y = float(self.last_vy)
        msg.angular.z = float(self.last_wz)

        # FACE THE PATH: Override heading to prioritize rotation before movement
        if self.face_movement and self.waypoints:
            lookahead_idx = min(15, len(self.waypoints) - 1)
            target = self.waypoints[lookahead_idx]
            target_yaw = math.atan2(target.y - ry, target.x - rx)
            err = target_yaw - yaw
            while err > math.pi: err -= 2*math.pi
            while err < -math.pi: err += 2*math.pi

            # Override angular with strong direct heading controller
            msg.angular.z = max(min(2.0 * err, 1.5), -1.5)

            # Scale linear down by alignment: cos(0)=1 full speed, cos(90deg)=0 stop, anything behind=0
            alignment = max(0.0, math.cos(err))
            msg.linear.x *= alignment
            msg.linear.y *= alignment

        self.cmd_vel_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(DWAPlannerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

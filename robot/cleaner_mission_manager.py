import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
import time, math

class MissionManager(Node):
    def __init__(self):
        super().__init__('cleaner_mission_manager')
        self.get_logger().info("🎯 OMEGA-TRACKING ACTIVE. Final Fix Deploying.")
        
        # Hardware Constraints
        self.L1, self.L2 = 0.12, 0.12
        self.pedestal_offset_x = 0.10
        self.pedestal_height = 0.135
        
        # State Data
        self.declare_parameter('world_name', 'mini_proving_ground')
        self.world = self.get_parameter('world_name').get_parameter_value().string_value
        
        self.state = 'IDLE' 
        self.is_processing = False
        self.current_pose = None
        self.cube_pose = None
        self.cube_seen = False
        
        # Set Default Goals based on Map
        self.default_goal = {'x': 10.0, 'y': 0.0} # Default finish line
        if 'benchmark' in self.world: self.default_goal = {'x': 4.0, 'y': 0.0}
        elif 'gauntlet' in self.world: self.default_goal = {'x': 10.0, 'y': 0.0}
        elif 'proving' in self.world: self.default_goal = {'x': 1.5, 'y': 1.5}
        
        self.get_logger().info(f"🌍 Map: {self.world} | Default Goal: {self.default_goal}")
        
        # Comms
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.j1_pub = self.create_publisher(Float64, '/arm/j1/cmd_pos', 10)
        self.j2_pub = self.create_publisher(Float64, '/arm/j2/cmd_pos', 10)
        self.grab_pub = self.create_publisher(Bool, '/mission/grab_control', 10)
        
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Pose, '/model/entity_omega_99/pose', self.cube_cb, 10)
        self.create_subscription(Pose, '/model/target_alpha/pose', self.cube_cb, 10)
        
        self.timer = self.create_timer(1.0, self.loop)
        self.start_time = time.time()

    def odom_cb(self, msg): self.current_pose = msg.pose.pose
    def cube_cb(self, msg): 
        # Update dynamic target pose
        if self.cube_pose is None: self.cube_pose = Pose()
        self.cube_pose = msg
        if not self.cube_seen:
            self.get_logger().info(f"👁️ TARGET SPOTTED at {msg.position.x:.2f}, {msg.position.y:.2f}! Starting Mission.")
            self.state = 'DRIVE_TO_BLOCK'
        self.cube_seen = True

    def solve_ik(self, x, z):
        dist_sq = x**2 + z**2
        dist = math.sqrt(dist_sq)
        if dist > (self.L1 + self.L2): return 1.6, 1.0 # Extended reach
        cos_theta2 = (dist_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        theta2 = math.acos(max(-1.0, min(1.0, cos_theta2)))
        theta1 = math.atan2(z, x) - math.atan2(self.L2 * math.sin(theta2), self.L1 + self.L2 * math.cos(theta2))
        return (math.pi/2) - theta1, theta2

    def loop(self):
        if self.current_pose is None or self.is_processing: return
        
        # Fallback Logic: If no cube seen for 5s, use default world goal
        if self.cube_pose is None:
            if time.time() - self.start_time > 5.0 and self.state == 'IDLE':
                self.get_logger().info(f"🛰️ No dynamic target found. Engaging world benchmark goal: {self.default_goal}")
                self.cube_pose = Pose()
                self.cube_pose.position.x = float(self.default_goal['x'])
                self.cube_pose.position.y = float(self.default_goal['y'])
                self.state = 'DRIVE_TO_BLOCK'
            return

        rx, ry = self.current_pose.position.x, self.current_pose.position.y
        cx, cy = self.cube_pose.position.x, self.cube_pose.position.y
        
        if self.state == 'IDLE':
            self.state = 'DRIVE_TO_BLOCK'
            return 
            
        # ANTI-SLIDE SAFETY: Spam Magnet OFF until we arrive
        if self.state == 'DRIVE_TO_BLOCK':
            self.grab_pub.publish(Bool(data=False))
            
            # Smart Standoff: 15cm from target center (-X assumes we face +X)
            target_stop_x = cx - 0.16 
            target_stop_y = cy
            self.send_goal({'x': target_stop_x, 'y': target_stop_y})
            
            dist_to_target = math.sqrt((rx - target_stop_x)**2 + (ry - target_stop_y)**2)
            if dist_to_target < 0.10:
                if self.cube_seen:
                    self.get_logger().info("🏆 [PICKUP] Target Acquired. Solving IK...")
                    self.state = 'PICKING_UP'
                    dx = cx - (rx + self.pedestal_offset_x)
                    dz = self.cube_pose.position.z - self.pedestal_height
                    j1, j2 = self.solve_ik(dx, dz)
                    self.perform_pickup(j1, j2)
                else:
                    self.get_logger().info("🏁 [SUCCESS] Benchmark Goal Reached.")
                    self.state = 'FINISH'

        elif self.state == 'DRIVE_TO_DUMPSTER':
            self.send_goal({'x': 0.0, 'y': 0.0})
            dist_to_home = math.sqrt(rx**2 + ry**2)
            if dist_to_home < 0.15:
                self.get_logger().info("🏁 [SUCCESS] Garbage Deposited.")
                self.grab_pub.publish(Bool(data=False))
                self.state = 'FINISH'

    def send_goal(self, pos):
        m = PoseStamped(); m.header.frame_id = "odom"
        m.pose.position.x, m.pose.position.y, m.pose.orientation.w = float(pos['x']), float(pos['y']), 1.0
        self.goal_pub.publish(m)

    def perform_pickup(self, j1, j2):
        self.is_processing = True
        self.j1_pub.publish(Float64(data=j1))
        self.j2_pub.publish(Float64(data=j2))
        time.sleep(4.0) # WAIT FOR STABILIZATION
        
        self.get_logger().info("🧲 GRIP ENGAGED.")
        self.grab_pub.publish(Bool(data=True))
        time.sleep(2.0)
        
        self.j1_pub.publish(Float64(data=0.0))
        self.j2_pub.publish(Float64(data=0.0))
        time.sleep(3.0)
        self.state = 'DRIVE_TO_DUMPSTER'
        self.is_processing = False

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': main()

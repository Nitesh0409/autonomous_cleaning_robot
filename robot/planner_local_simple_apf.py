import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

class SimpleAPFNode(Node):
    def __init__(self):
        super().__init__('simple_apf')
        
        # Parameters
        self.declare_parameter('k_att', 1.0)  # Attractive gain
        self.declare_parameter('k_rep', 0.1)  # Repulsive gain
        self.declare_parameter('d0', 1.0)     # Obstacle influence distance
        self.declare_parameter('max_vel', 0.5)
        
        # Pubs/Subs
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        self.odom = None
        self.scan = None
        self.goal = None
        
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Simple APF Node Initialized.")

    def odom_callback(self, msg): self.odom = msg
    def scan_callback(self, msg): self.scan = msg
    def goal_callback(self, msg): 
        self.goal = msg.pose.position
        self.get_logger().info(f"New Goal: {self.goal.x}, {self.goal.y}")

    def control_loop(self):
        if not self.odom or not self.goal: return
        
        # 1. Pose & Goal Vector
        pose = self.odom.pose.pose
        dx = self.goal.x - pose.position.x
        dy = self.goal.y - pose.position.y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < 0.2:
            self.pub_vel.publish(Twist())
            return

        # 2. Attractive Force
        k_att = self.get_parameter('k_att').value
        f_att_x = k_att * (dx / dist)
        f_att_y = k_att * (dy / dist)
        
        # 3. Repulsive Force
        f_rep_x, f_rep_y = 0.0, 0.0
        if self.scan:
            k_rep = self.get_parameter('k_rep').value
            d0 = self.get_parameter('d0').value
            ranges = np.array(self.scan.ranges)
            angles = np.linspace(self.scan.angle_min, self.scan.angle_max, len(ranges))
            
            # Extract robot yaw for coordination
            q = pose.orientation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y**2 + q.z**2))
            
            for r, a in zip(ranges, angles):
                if 0.1 < r < d0:
                    # Magnitude of repulsion
                    rep_mag = k_rep * (1.0/r - 1.0/d0) * (1.0/r**2)
                    
                    # Convert Lidar relative angle to World Frame
                    world_a = a + yaw
                    f_rep_x -= rep_mag * math.cos(world_a)
                    f_rep_y -= rep_mag * math.sin(world_a)

        # 4. Total Force & Command
        total_x = f_att_x + f_rep_x
        total_y = f_att_y + f_rep_y
        
        # Limit velocity
        max_vel = self.get_parameter('max_vel').value
        mag = math.sqrt(total_x**2 + total_y**2)
        if mag > max_vel:
            total_x = (total_x / mag) * max_vel
            total_y = (total_y / mag) * max_vel
            
        # Convert total world force to body frame for publication
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        vx_body = total_x * cos_y + total_y * sin_y
        vy_body = -total_x * sin_y + total_y * cos_y
        
        cmd = Twist()
        cmd.linear.x = vx_body
        cmd.linear.y = vy_body
        
        # Simple rotation toward goal
        goal_yaw = math.atan2(dy, dx)
        yaw_err = goal_yaw - yaw
        while yaw_err > math.pi: yaw_err -= 2*math.pi
        while yaw_err < -math.pi: yaw_err += 2*math.pi
        cmd.angular.z = np.clip(1.5 * yaw_err, -1.0, 1.0)
        
        self.pub_vel.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(SimpleAPFNode())
    rclpy.shutdown()

if __name__ == '__main__': main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import time
import sys

class NavTester(Node):
    def __init__(self):
        super().__init__('nav_tester')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_pose = None
        self.goal_reached = False
        self.target_x = 2.5
        self.target_y = -0.5
        self.start_time = None
        
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        
        # Check distance to target
        dx = self.current_pose.position.x - self.target_x
        dy = self.current_pose.position.y - self.target_y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < 0.3: # Threshold
            self.get_logger().info(f"SUCCESS: Goal Reached! Final Dist: {dist:.2f}m")
            self.goal_reached = True
            
    def send_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        self.goal_pub.publish(msg)
        self.get_logger().info(f"BENCHMARK: Sent goal to ({self.target_x}, {self.target_y})")
        self.start_time = time.time()

def main():
    rclpy.init()
    node = NavTester()
    
    # Wait for sim to be ready (odom data flowing)
    print("Waiting for Odometry data...")
    while rclpy.ok() and node.current_pose is None:
        rclpy.spin_once(node, timeout_sec=1.0)
        
    print("Simulation Ready. Starting Headless Benchmark...")
    node.send_goal()
    
    try:
        last_log_time = 0
        while rclpy.ok() and not node.goal_reached:
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Periodic Progress Log (1Hz)
            now = time.time()
            if now - last_log_time > 1.0 and node.current_pose:
                dx = node.current_pose.position.x - node.target_x
                dy = node.current_pose.position.y - node.target_y
                dist = math.sqrt(dx**2 + dy**2)
                print(f"[REPORTER] Pos: ({node.current_pose.position.x:.2f}, {node.current_pose.position.y:.2f}) | Dist to Goal: {dist:.2f}m")
                last_log_time = now
                
            # Timeout after 60s
            if node.start_time and (time.time() - node.start_time > 60):
                print("FAILURE: Navigation Timeout - Robot failed to reach goal in 60s.")
                sys.exit(1)
                
        if node.goal_reached:
            print("Test Passed Successfully.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

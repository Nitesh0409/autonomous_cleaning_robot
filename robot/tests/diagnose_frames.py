import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import sys

class FrameDiagnoser(Node):
    def __init__(self):
        super().__init__('frame_diagnoser')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.start_pose = None
        self.current_pose = None
        
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.start_pose is None:
            self.start_pose = msg.pose.pose

    def run_diagnostics(self):
        print("Waiting for Odometry...")
        while rclpy.ok() and self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        # Test X-Axis
        print("\n--- TESTING X-AXIS ---")
        self.start_pose = self.current_pose
        msg = Twist()
        msg.linear.x = 0.5
        for _ in range(20):
            self.cmd_pub.publish(msg)
            time.sleep(0.1)
        self.cmd_pub.publish(Twist()) # Stop
        time.sleep(1.0)
        dx = self.current_pose.position.x - self.start_pose.position.x
        dy = self.current_pose.position.y - self.start_pose.position.y
        print(f"Command linear.x=0.5 caused World Movement: dx={dx:.4f}, dy={dy:.4f}")

        # Test Y-Axis
        print("\n--- TESTING Y-AXIS ---")
        self.start_pose = self.current_pose
        msg = Twist()
        msg.linear.y = 0.5
        for _ in range(20):
            self.cmd_pub.publish(msg)
            time.sleep(0.1)
        self.cmd_pub.publish(Twist()) # Stop
        time.sleep(1.0)
        dx = self.current_pose.position.x - self.start_pose.position.x
        dy = self.current_pose.position.y - self.start_pose.position.y
        print(f"Command linear.y=0.5 caused World Movement: dx={dx:.4f}, dy={dy:.4f}")
        
        print("\nDIAGNOSTICS COMPLETE.")

def main():
    rclpy.init()
    node = FrameDiagnoser()
    try:
        node.run_diagnostics()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

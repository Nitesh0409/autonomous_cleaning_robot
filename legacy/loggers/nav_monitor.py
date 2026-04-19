import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math, os
from datetime import datetime

class NavMonitorNode(Node):
    def __init__(self):
        super().__init__('nav_monitor')
        
        # Paths for telemetry logging - Robust check
        ws_root = os.getcwd()
        self.log_dir = os.path.join(ws_root, 'loggers', 'logs')
        
        if os.path.isfile(self.log_dir):
            os.remove(self.log_dir)
        os.makedirs(self.log_dir, exist_ok=True)
        self.telemetry_file = os.path.join(self.log_dir, f"telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        
        with open(self.telemetry_file, 'w') as f:
            f.write("timestamp,odom_dist,truth_dist,drift_percent\n")

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.truth_sub = self.create_subscription(Odometry, '/ground_truth', self.truth_callback, 10)
        
        self.odom_pose = None
        self.truth_pose = None
        self.start_odom = None
        self.start_truth = None
        
        self.timer = self.create_timer(1.0, self.log_telemetry)
        self.get_logger().info(f"📊 MONITOR ACTIVE. Continuous logs at: {self.telemetry_file}")

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        if self.start_odom is None: self.start_odom = msg.pose.pose.position

    def truth_callback(self, msg):
        self.truth_pose = msg.pose.pose
        if self.start_truth is None: self.start_truth = msg.pose.pose.position

    def get_dist(self, p1, p2):
        if p1 is None or p2 is None: return 0.0
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def log_telemetry(self):
        if self.odom_pose is None or self.truth_pose is None: return

        d_odom = self.get_dist(self.odom_pose.position, self.start_odom)
        d_truth = self.get_dist(self.truth_pose.position, self.start_truth)
        
        error = ((d_odom - d_truth) / d_truth) * 100.0 if d_truth > 0.01 else 0.0
        
        # Save to CSV
        with open(self.telemetry_file, 'a') as f:
            f.write(f"{self.get_clock().now().to_msg().sec},{d_odom:.3f},{d_truth:.3f},{error:.1f}\n")
            
        self.get_logger().info(f"📍 RViz: {d_odom:.2f}m | Gazebo: {d_truth:.2f}m | Drift: {error:+.1f}%", throttle_duration_sec=2.0)

def main():
    rclpy.init()
    rclpy.spin(NavMonitorNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

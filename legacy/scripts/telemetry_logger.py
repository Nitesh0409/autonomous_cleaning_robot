import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os
import time

class TelemetryLogger(Node):
    def __init__(self):
        super().__init__('telemetry_logger')
        # Precise timestamped log for calibration
        self.log_file = os.path.join(os.getcwd(), f'telemetry_log_v75_{int(time.time())}.csv')
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        with open(self.log_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'pos_x', 'pos_y', 'yaw', 'vel_x', 'vel_y', 'vel_omega'])
            
        self.get_logger().info(f'🚀 Nav Log Active: {self.log_file}')

    def odom_callback(self, msg):
        ts = self.get_clock().now().to_msg().sec + (self.get_clock().now().to_msg().nanosec / 1e9)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        # Yaw extraction from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        import math
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vo = msg.twist.twist.angular.z
        
        with open(self.log_file, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([ts, px, py, yaw, vx, vy, vo])

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

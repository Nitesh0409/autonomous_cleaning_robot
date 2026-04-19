import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion
import csv
import os
import time

class DriftAnalyzer(Node):
    def __init__(self):
        super().__init__('drift_analyzer')
        # Subscribe to ground truth (Gazebo) and odom (ROS2)
        self.gt_sub = self.create_subscription(Odometry, '/model/robot/ground_truth', self.gt_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.gt_msg = None
        self.odom_msg = None
        # Prepare CSV log file
        log_dir = os.path.join(os.getenv('HOME'), 'drift_logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(log_dir, f'drift_{timestamp}.csv')
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'gt_x', 'gt_y', 'gt_yaw', 'odom_x', 'odom_y', 'odom_yaw', 'dx', 'dy', 'dyaw'])
        self.get_logger().info(f'Drift log will be written to {self.csv_path}')

    def gt_callback(self, msg: Odometry):
        self.gt_msg = msg
        self.try_compute()

    def odom_callback(self, msg: Odometry):
        self.odom_msg = msg
        self.try_compute()

    def try_compute(self):
        if self.gt_msg is None or self.odom_msg is None:
            return
        # Extract positions
        gt_pose = self.gt_msg.pose.pose
        odom_pose = self.odom_msg.pose.pose
        gt_x = gt_pose.position.x
        gt_y = gt_pose.position.y
        odom_x = odom_pose.position.x
        odom_y = odom_pose.position.y
        # Extract yaw from quaternion
        def yaw_from_quat(q):
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return yaw
        gt_yaw = yaw_from_quat(gt_pose.orientation)
        odom_yaw = yaw_from_quat(odom_pose.orientation)
        # Compute differences
        dx = gt_x - odom_x
        dy = gt_y - odom_y
        dyaw = gt_yaw - odom_yaw
        # Log to CSV
        now = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([now, gt_x, gt_y, gt_yaw, odom_x, odom_y, odom_yaw, dx, dy, dyaw])
            
        # Log to terminal
        self.get_logger().info(
            f'Drift → dx: {dx:.3f} m, dy: {dy:.3f} m, dyaw: {dyaw:.3f} rad'
        )
        # Reset messages to avoid duplicate rows (optional)
        self.gt_msg = None
        self.odom_msg = None

def main(args=None):
    rclpy.init(args=args)
    node = DriftAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

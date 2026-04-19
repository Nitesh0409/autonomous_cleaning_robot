import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class CalibrationScorer(Node):
    """
    A diagnostic node to measure the 'Reality Gap' between Gazebo and RViz.
    It compares the Ground Truth Odom with the TF tree to calculate absolute drift.
    """
    def __init__(self):
        super().__init__('calib_verify')
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0, self.check_precision)
        self.latest_odom = None
        self.get_logger().info("Calibration Scorer Started. Monitoring for drift...")

    def odom_callback(self, msg):
        self.latest_odom = msg.pose.pose

    def check_precision(self):
        if self.latest_odom is None:
            self.get_logger().warn("Waiting for /odom data...")
            return

        try:
            # Look up the transform from odom to base_link
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', now)
            
            # Compare Gazebo Odom (Truth) with TF (Visualization)
            dx = self.latest_odom.position.x - trans.transform.translation.x
            dy = self.latest_odom.position.y - trans.transform.translation.y
            
            error_dist = math.sqrt(dx**2 + dy**2)
            
            # Calibration Quality Metrics
            if error_dist < 0.001:
                quality = "🏆 PIXEL PERFECT"
            elif error_dist < 0.005:
                quality = "✅ HIGH PRECISION"
            elif error_dist < 0.02:
                quality = "⚠️ FAIR (Drift Detected)"
            else:
                quality = "❌ POOR (Calibration Required)"

            self.get_logger().info(f"\n" + "-"*40 + 
                                   f"\n[CALIBRATION SCORE] {quality}" +
                                   f"\nAbsolute Drift: {error_dist*1000:.3f} mm" +
                                   f"\nCalculated Error: ΔX:{dx*1000:.3f}mm, ΔY:{dy*1000:.3f}mm" +
                                   f"\n" + "-"*40)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"TF Lookup Failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationScorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

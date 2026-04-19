import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time
import math

class StaticBenchmarkTests(Node):
    def __init__(self):
        super().__init__('benchmark_runner')
        
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.current_pose = None
        self.start_timer = None
        
        # Test 1: Simply cross the centerline to reach (10.0, 0.0)
        # The slalom_gauntlet.sdf has obstacles distributed up to 8m.
        self.target_x = 10.0
        self.target_y = 0.0
        
        self.has_started = False
        
        self.get_logger().info("🔥 GAUNTLET RUNNER ONLINE: Preparing for slalom...")
        self.timer = self.create_timer(0.5, self.benchmark_loop)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def get_distance(self, tx, ty):
        if not self.current_pose: return 999.0
        return math.sqrt((self.current_pose.position.x - tx)**2 + (self.current_pose.position.y - ty)**2)

    def benchmark_loop(self):
        if self.current_pose is None:
            return
            
        # Initialize test when odometry is valid
        if not self.has_started:
            self.target_y = 3.0 # Centerline of the Gauntlet
            self.get_logger().info("🎯 Odom stable. Transmitting Gauntlet Goal Coordinate (-10.0, 3.0)...")
            
            goal = PoseStamped()
            goal.header.frame_id = "odom"
            goal.pose.position.x = 10.0
            goal.pose.position.y = 3.0
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            
            self.goal_pub.publish(goal)
            self.start_timer = time.time()
            self.has_started = True
            return
            
        # Monitor progress
        elapsed = time.time() - self.start_timer
        dist = self.get_distance(self.target_x, self.target_y)
        
        if dist < 0.15:
            self.get_logger().info(f"🏆 SUCCESS! Strafe & Pass completed in {elapsed:.2f} seconds!")
            self.get_logger().info(f"Test Over. Standing by for manual intervention.")
            self.destroy_timer(self.timer)
        elif elapsed > 30.0:
            self.get_logger().error(f"❌ TIMEOUT! Robot took > 30s to clear obstacle.")
            self.destroy_timer(self.timer)
        else:
            self.get_logger().info(f"⏱️ Running... Dist: {dist:.2f}m | Time: {elapsed:.1f}s")


def main():
    rclpy.init()
    node = StaticBenchmarkTests()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

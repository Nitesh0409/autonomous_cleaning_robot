import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped

class MissionHealthProbe(Node):
    def __init__(self):
        super().__init__('mission_health_probe')
        self.health = {
            'odom': 0,
            'front_scan': 0,
            'rear_scan': 0,
            'goal': 0,
            'cmd_vel': 0,
            'plan': 0
        }
        
        # Namespaced Subscribers
        self.create_subscription(Odometry, '/unit_sigma_01/odom', lambda m: self.update('odom'), 10)
        self.create_subscription(LaserScan, '/unit_sigma_01/front_scan', lambda m: self.update('front_scan'), 10)
        self.create_subscription(LaserScan, '/unit_sigma_01/rear_scan', lambda m: self.update('rear_scan'), 10)
        self.create_subscription(PoseStamped, '/unit_sigma_01/goal_pose', lambda m: self.update('goal'), 10)
        self.create_subscription(Twist, '/unit_sigma_01/cmd_vel', lambda m: self.update('cmd_vel'), 10)
        self.create_subscription(Path, '/unit_sigma_01/plan', lambda m: self.update('plan'), 10)
        
        self.timer = self.create_timer(1.0, self.report)
        print("🔍 MISSION HEALTH PROBE ACTIVE. Monitoring 'unit_sigma_01' domain...")

    def update(self, key):
        self.health[key] += 1

    def report(self):
        print("\n--- Telemetry Pulse ---")
        for k, v in self.health.items():
            status = "✅ OK" if v > 0 else "❌ SILENT"
            print(f"{k.upper():<12}: {v:<5} reports | {status}")
        
        if self.health['odom'] == 0:
            print("⚠️ FATAL: Odometry is SILENT. Bridge or Physics stall.")
        if self.health['goal'] > 0 and self.health['plan'] == 0:
            print("⚠️ ERROR: Goal received but Planner is NOT calculating Path.")
        if self.health['plan'] > 0 and self.health['cmd_vel'] == 0:
            print("⚠️ ERROR: Path exists but Local Planner is NOT driving wheels.")

def main():
    rclpy.init()
    node = MissionHealthProbe()
    try: rclpy.spin(node)
    except: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time
import math
import json
import os
import subprocess

class MissionSuiteRunner(Node):
    """
    Automated Test Pilot (Suite Mode):
    1. Loads missions from config/nav_tests.json.
    2. Teleports robot to start.
    3. Sends goal and monitors performance.
    4. Advances to next mission on success/timeout.
    """
    def __init__(self, config_file):
        super().__init__('orchestrate_test')
        self.config_file = config_file
        self.scenarios = []
        self.current_scenario_idx = 0
        self.load_config()

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/mission_goal_marker', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.current_pose = None
        self.mission_active = False
        self.start_timer = None
        self.results = []
        
        self.get_logger().info(f"Test Suite Runner Online. {len(self.scenarios)} scenarios loaded.")
        self.timer = self.create_timer(1.0, self.suite_control_loop)

    def load_config(self):
        try:
            with open(self.config_file, 'r') as f:
                data = json.load(f)
                self.scenarios = data.get('scenarios', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def suite_control_loop(self):
        if self.current_pose is None:
            self.get_logger().warn("Waiting for sensors (/odom)...")
            return

        if not self.mission_active:
            if self.current_scenario_idx < len(self.scenarios):
                self.start_mission()
            else:
                self.print_summary()
                self.get_logger().info("--- ALL TEST SCENARIOS COMPLETE ---")
                self.destroy_node()
                rclpy.shutdown()
            return

        # Monitor active mission
        scenario = self.scenarios[self.current_scenario_idx]
        target = scenario['goal']
        dist = self.calculate_distance(target['x'], target['y'])
        elapsed = time.time() - self.start_timer
        
        if dist < 0.2:
            self.log_result(scenario['name'], "SUCCESS", elapsed)
            self.end_mission()
        elif elapsed > 45.0:
            self.log_result(scenario['name'], "TIMEOUT", elapsed)
            self.end_mission()
        else:
            self.get_logger().info(f"🛰️ [{scenario['name']}] Dist: {dist:.2f}m | Time: {elapsed:.1f}s")

    def start_mission(self):
        scenario = self.scenarios[self.current_scenario_idx]
        self.get_logger().info(f"🏁 Starting Scenario: {scenario['name']}")
        
        # 1. Teleport Robot
        self.teleport_robot(scenario['start'])
        time.sleep(1) # Wait for simulation to settle
        
        # 2. Send Goal
        self.send_goal(scenario['goal'])
        self.start_timer = time.time()
        self.mission_active = True

    def end_mission(self):
        self.mission_active = False
        self.current_scenario_idx += 1
        time.sleep(2) # Cooldown between tests

    def teleport_robot(self, start):
        """Uses gz service to reset the robot's pose."""
        self.get_logger().info(f"Teleporting robot to ({start['x']}, {start['y']})")
        # Note: We use subprocess to call the gz service command discovered in reset_robot.sh
        cmd = [
            'gz', 'service', '-s', '/world/mini_proving_ground/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--req', f'name: "robot", position: {{x: {start["x"]}, y: {start["y"]}, z: 0.1}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
        ]
        try:
            subprocess.run(cmd, check=True, capture_output=True)
        except Exception as e:
            self.get_logger().error(f"Teleport command failed: {e}")

    def send_goal(self, goal):
        # 1. Pub PoseStamped
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = float(goal['x'])
        msg.pose.position.y = float(goal['y'])
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

        # 2. Pub Visual Marker
        from visualization_msgs.msg import Marker
        from std_msgs.msg import ColorRGBA
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "mission_goal"; m.id = 0
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = msg.pose.position.x, msg.pose.position.y, 0.5
        m.scale.x, m.scale.y, m.scale.z = 0.3, 0.3, 0.3
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8) # Solid Green
        self.marker_pub.publish(m)

    def calculate_distance(self, target_x, target_y):
        dx = target_x - self.current_pose.position.x
        dy = target_y - self.current_pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def log_result(self, name, status, time_taken):
        emoji = "✅" if status == "SUCCESS" else "❌"
        self.results.append({'name': name, 'status': status, 'time': time_taken})
        self.get_logger().info(f"{emoji} {name} result: {status} in {time_taken:.1f}s")

    def print_summary(self):
        print("\n" + "="*50)
        print("          NAVIGATION TEST SUITE SUMMARY")
        print("="*50)
        for r in self.results:
            status_text = f"{r['status']:<8}"
            print(f"{r['name']:<25} | {status_text} | {r['time']:.1f}s")
        print("="*50 + "\n")

def main(args=None):
    rclpy.init(args=args)
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.join(root_dir, 'config', 'nav_tests.json')
    node = MissionSuiteRunner(config_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

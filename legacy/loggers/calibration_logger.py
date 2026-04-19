#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time, math, threading, os
from datetime import datetime

class CalibrationLogger(Node):
    def __init__(self):
        super().__init__('calibration_logger')
        
        # 1. Paths for saving - Robust check (Fixing ROS2 install dir bug)
        ws_root = os.getcwd()
        self.log_dir = os.path.join(ws_root, 'loggers', 'logs')
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = os.path.join(self.log_dir, f"calibration_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")

        # 2. ROS setup
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/ground_truth', self.truth_cb, 10)
        
        self.pose = None            
        self.truth_pose = None
        self.start_pose = None      
        self.start_truth = None
        self.results = [] 

        self.target_dist = 0.0      
        self.step_speed = Twist()   
        self.step_active = False    
        self.step_name = ""         

        # High-frequency control loop (100Hz) to ensure no missed messages
        self.create_timer(0.01, self.control_loop)
        self.get_logger().info(f"📊 LOGGER READY. Recording to: {self.log_file}")
        
        threading.Thread(target=self.run_sequence_thread, daemon=True).start()

    def odom_cb(self, msg): self.pose = msg.pose.pose
    def truth_cb(self, msg): self.truth_pose = msg.pose.pose

    def _get_dist(self, curr, start, is_rotation=False):
        if curr is None or start is None: return 0.0
        if is_rotation:
            q_s, q_c = start.orientation, curr.orientation
            y_s = math.atan2(2*(q_s.w*q_s.z + q_s.x*q_s.y), 1 - 2*(q_s.y**2 + q_s.z**2))
            y_c = math.atan2(2*(q_c.w*q_c.z + q_c.x*q_c.y), 1 - 2*(q_c.y**2 + q_c.z**2))
            diff = y_c - y_s
            while diff > math.pi: diff -= 2*math.pi
            while diff < -math.pi: diff += 2*math.pi
            return abs(math.degrees(diff))
        else:
            dx, dy = curr.position.x - start.position.x, curr.position.y - start.position.y
            return math.sqrt(dx**2 + dy**2)

    def control_loop(self):
        if not self.step_active: return
        
        # 🚀 [MOTION ENGINE: PUBLISH]
        # Always publish even if odom is missing (Blind Drive)
        self.cmd_pub.publish(self.step_speed)
        
        if self.pose is not None and self.start_pose is not None:
            is_rot = 'Turn' in self.step_name
            progress = self._get_dist(self.pose, self.start_pose, is_rot)
            
            if progress >= self.target_dist:
                self.cmd_pub.publish(Twist())
                t_dist = self._get_dist(self.truth_pose, self.start_truth, is_rot) if self.truth_pose else 0.0
                error = ((progress - t_dist) / t_dist) * 100.0 if t_dist > 0 else 0.0
                
                # PARASITIC TRANSLATION FIX
                if is_rot and self.truth_pose and self.start_truth:
                    dx = self.truth_pose.position.x - self.start_truth.position.x
                    dy = self.truth_pose.position.y - self.start_truth.position.y
                    translation_drift = math.sqrt(dx**2 + dy**2)
                    # Punish the score heavily (e.g., 0.1m XY slip = +15% error penalty)
                    penalty = (translation_drift / 0.1) * 15.0
                    error += penalty
                    self.get_logger().info(f"⚠️ Parasitic Translation detected: {translation_drift:.3f}m -> Penalty Added: +{penalty:.1f}%")
                
                self.results.append((self.step_name, progress, t_dist, error, "°" if is_rot else "m"))
                self.step_active = False

    def _start_step(self, name, cmd, target):
        self.get_logger().info(f"🚀 EXECUTING: {name} (0 / {target})")
        self.start_pose, self.start_truth = self.pose, self.truth_pose
        self.step_name, self.target_dist, self.step_speed, self.step_active = name, target, cmd, True
        
        start_time = time.time()
        while self.step_active and rclpy.ok():
            if time.time() - start_time > 15.0: # Fail-safe
                self.get_logger().error(f"❌ TIMEOUT: {name}")
                self.cmd_pub.publish(Twist())
                self.step_active = False
            time.sleep(0.1)

    def run_sequence_thread(self):
        time.sleep(3.0)
        tasks = [("Forward 1.0m", 0.3, 0.0, 0.0, 1.0), ("Turn 90.0°", 0.0, 0.0, 0.5, 90.0), ("Strafe 0.5m", 0.0, 0.3, 0.0, 0.5)]
        import sys
        
        if "--linear" in sys.argv: tasks = [tasks[0]]
        elif "--rotation" in sys.argv: tasks = [tasks[1]]
        elif "--strafe" in sys.argv: tasks = [tasks[2]]
        
        for n, vx, vy, wz, t in tasks:
            if "--auto" not in sys.argv:
                input(f"\n▶️ Press [ENTER] to start: {n}")
            else:
                self.get_logger().info(f"\n▶️ AUTO STARTING: {n}")
                time.sleep(1.0)
            cmd = Twist(); cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
            self._start_step(n, cmd, t)

        report = f"\nCALIBRATION FINAL REPORT\n" + "="*50 + "\n"
        report += f"{'TEST':<15} | {'ODOM':<10} | {'ACTUAL':<10} | {'DRIFT'}\n"
        for n, o, tr, e, u in self.results:
            report += f"{n:<15} | {o:7.3f}{u} | {tr:7.3f}{u} | {e:+.1f}%\n"
        print(report)
        with open(self.log_file, 'w') as f: 
            f.write(report)
            f.flush()
            os.fsync(f.fileno()) # Force Windows to sync file
        
        self.get_logger().info("✅ CALIBRATION COMPLETE. Exiting...")
        time.sleep(1.0)
        os._exit(0) # Force hard exit to release all ROS resources immediately

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

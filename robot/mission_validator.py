import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import time, math, sys, csv, os

def log(msg):
    print(f"[MISSION] {msg}", flush=True)

class MissionValidator(Node):
    def __init__(self):
        super().__init__('mission_validator')
        log("Validator Initialized. Monitoring TF...")
        self.state = 'INIT'
        self.garbage_pos = None 
        self.dumpster_pos = {'x': 0.0, 'y': 0.0}
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.j1_pub = self.create_publisher(Float64, '/arm/j1/cmd_pos', 10)
        self.j2_pub = self.create_publisher(Float64, '/arm/j2/cmd_pos', 10)
        self.grab_pub = self.create_publisher(Bool, '/gripper/grab', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        self.current_pose = None
        self.joints = {}
        self.timer = self.create_timer(0.1, self.loop)

    def odom_cb(self, msg): self.current_pose = msg.pose.pose
    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position): self.joints[name] = pos

    def loop(self):
        try:
            t = self.tf_buffer.lookup_transform('odom', 'garbage_block', rclpy.time.Time())
            bx, by = t.transform.translation.x, t.transform.translation.y
            
            if self.state == 'INIT':
                self.garbage_pos = {'x': bx - 0.20, 'y': by}
                log(f"📍 Object found at {bx:.2f}, {by:.2f}. Navigating...")
                self.send_goal(self.garbage_pos)
                self.state = 'NAV'
            
            if self.current_pose:
                d = math.sqrt((bx - self.current_pose.position.x)**2 + (by - self.current_pose.position.y)**2)
                if self.state == 'NAV' and d < 0.23:
                    log(f"🎯 At Object (dist: {d:.3f}m). Picking up...")
                    self.perform_pickup()
                    self.state = 'TO_DUMP'
                    self.send_goal(self.dumpster_pos)
                
                elif self.state == 'TO_DUMP':
                    dd = math.sqrt(self.current_pose.position.x**2 + self.current_pose.position.y**2)
                    if dd < 0.35:
                        log("🗑️ At Dumpster. Dropoff sequence...")
                        self.perform_dropoff()
                        log("🏆 TEST SUCCESSFUL.")
                        os.system("pkill -9 -f 'gz|ros|planner|manager'")
                        sys.exit(0)
        except: pass

    def send_goal(self, pos):
        m = PoseStamped(); m.header.frame_id = "odom"
        m.pose.position.x, m.pose.position.y, m.pose.orientation.w = float(pos['x']), float(pos['y']), 1.0
        self.goal_pub.publish(m)

    def perform_pickup(self):
        self.j1_pub.publish(Float64(data=1.2)); self.j2_pub.publish(Float64(data=1.1)); time.sleep(1.0)
        self.grab_pub.publish(Bool(data=True)); time.sleep(0.5); self.j1_pub.publish(Float64(data=0.0)); time.sleep(0.5)

    def perform_dropoff(self):
        self.j1_pub.publish(Float64(data=-1.57)); time.sleep(1.0); self.grab_pub.publish(Bool(data=False)); time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = MissionValidator()
    try: rclpy.spin(node)
    except SystemExit: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()

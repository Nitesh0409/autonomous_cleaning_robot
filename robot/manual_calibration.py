import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
import time, math, sys, os
from tf2_ros import Buffer, TransformListener

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('arm_calibration_tool')
        self.get_logger().info("🛠️ MANUAL CALIBRATION TOOL ACTIVE.")
        self.get_logger().info("Use: 'ros2 param set /arm_calibration_tool arm_angle 0.85' to move arm.")
        
        # Parameters
        self.declare_parameter('arm_angle', 0.85)
        self.declare_parameter('gripper_grab', False)
        
        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.j1_pub = self.create_publisher(Float64, '/arm/j1/cmd_pos', 10)
        self.grab_pub = self.create_publisher(Bool, '/gripper/grab', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.current_pose = None
        self.timer = self.create_timer(0.5, self.loop)

    def odom_cb(self, msg): self.current_pose = msg.pose.pose

    def loop(self):
        if self.current_pose is None: return
        
        # 1. Drive to and Hold at Standoff (0.80)
        m = PoseStamped(); m.header.frame_id = "odom"
        m.pose.position.x = 0.80; m.pose.orientation.w = 1.0
        self.goal_pub.publish(m)
        
        # 2. Control Arm via Dynamic Parameters
        angle = self.get_parameter('arm_angle').value
        grab = self.get_parameter('gripper_grab').value
        self.j1_pub.publish(Float64(data=angle))
        self.grab_pub.publish(Bool(data=grab))
        
        # 3. Track origins and print "Truth Gap"
        try:
            tb = self.tf_buffer.lookup_transform('odom', 'garbage_block', rclpy.time.Time())
            te = self.tf_buffer.lookup_transform('odom', 'gripper_right', rclpy.time.Time())
            
            bx, by = tb.transform.translation.x, tb.transform.translation.y
            ex, ey = te.transform.translation.x, te.transform.translation.y
            gap = math.sqrt((bx-ex)**2 + (by-ey)**2) * 100
            
            self.get_logger().info(f"📏 TUNE: Angle {angle:.2f} | Gripper X:{ex:.2f} | Block X:{bx:.2f} | GAP: {gap:.1f}cm")
            
            # Markers
            mk = Marker(); mk.header.frame_id = "odom"; mk.id = 0; mk.type = Marker.SPHERE
            mk.pose.position.x, mk.pose.position.y = bx, by; mk.scale.x = mk.scale.y = mk.scale.z = 0.05
            mk.color.a = 1.0; mk.color.r = 1.0; self.marker_pub.publish(mk)
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': main()

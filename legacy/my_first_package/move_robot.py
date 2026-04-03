import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.move)

    def move(self):
        msg = Twist()
        msg.linear.x = 0.2   # forward speed
        msg.angular.z = 0.0  # no rotation
        self.publisher_.publish(msg)
        self.get_logger().info("Moving Forward 🚀")

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

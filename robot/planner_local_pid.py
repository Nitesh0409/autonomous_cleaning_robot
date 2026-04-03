import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GoToGoalNode(Node):
    def __init__(self):
        super().__init__('lino_simple')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_robot)
        self.get_logger().info('Lino Simple (Baseline) Node Started.')

    def move_robot(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

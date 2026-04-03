import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import numpy as np
import trajectory_predictor as tp


class MastermindNode(Node):
    def __init__(self):
        super().__init__('mastermind_node')
        # Use Gazebo model states instead of camera image — camera requires
        # OGRE rendering which crashes in headless (gzserver-only) mode on Windows.
        self.subscription = self.create_subscription(
            ModelStates,
            '/model_states',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.predictor = tp.TrajectoryPredictor()
        self.get_logger().info('Mastermind Node started, waiting for Gazebo model states...')

    def listener_callback(self, msg):
        try:
            ball_idx  = msg.name.index('trash_ball')
            robot_idx = msg.name.index('smart_dustbin')
        except ValueError:
            self.get_logger().warn('Models not yet in scene, waiting...')
            return

        ball_pos  = msg.pose[ball_idx].position
        robot_pos = msg.pose[robot_idx].position

        # Ball position relative to robot (ground plane = intercept target)
        error_x = ball_pos.x - robot_pos.x
        error_y = ball_pos.y - robot_pos.y
        dist_z  = ball_pos.z  # height above ground

        twist = Twist()

        if dist_z > 0.05:  # Ball is still in the air
            t_now = self.get_clock().now().nanoseconds / 1e9
            self.predictor.add_observation(t_now, error_x, error_y, dist_z)

            prediction = self.predictor.predict_landing()
            if prediction:
                x_land, y_land, t_land = prediction
                self.get_logger().info(f'Predicted Landing: X={x_land:.2f}, Y={y_land:.2f}')
                twist.linear.x  = float(np.clip(x_land * 0.5, -1.0, 1.0))
                twist.angular.z = float(np.clip(-y_land * 0.5, -1.0, 1.0))
            else:
                # Not enough observations yet — move toward ball
                twist.linear.x  = float(np.clip(error_x * 0.3, -1.0, 1.0))
                twist.angular.z = float(np.clip(-error_y * 0.3, -1.0, 1.0))
        else:
            self.get_logger().info('Ball on ground. Scanning...')
            twist.angular.z = 0.3

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    mastermind_node = MastermindNode()
    rclpy.spin(mastermind_node)
    mastermind_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Manual Control Mode (Arrow Keys)
---------------------------
UP    : Forward
DOWN  : Backward
LEFT  : Rotate Left
RIGHT : Rotate Right

CTRL-C to quit
"""

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(3)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = Node('arrow_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    print(msg)
    
    try:
        while True:
            key = getKey(settings)
            twist = Twist()
            
            # Arrow key escape sequences
            if key == '\x1b[A': # UP
                twist.linear.x = 0.5
            elif key == '\x1b[B': # DOWN
                twist.linear.x = -0.5
            elif key == '\x1b[D': # LEFT
                twist.angular.z = 1.0
            elif key == '\x1b[C': # RIGHT
                twist.angular.z = -1.0
            elif key == '\x03': # CTRL-C
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
            pub.publish(twist)
            
    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()

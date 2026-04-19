import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
import sys, select, termios, tty

# Instruction Manual
msg = """
--- ARM LAB CALIBRATION TOOL ---
Control your arm manually to find the perfect Grasp Angle.

[W / S] : Joint 1 (Shoulder) +/- 0.05 rad
[A / D] : Joint 2 (Elbow)    +/- 0.05 rad
[SPACE] : Toggle Gripper
[R]     : Reset to Home
[Q]     : Quit and Print Final Values

CURRENT COMMANDS:
"""

class ArmLabTuner(Node):
    def __init__(self):
        super().__init__('arm_lab_tuner')
        self.j1_pub = self.create_publisher(Float64, '/arm/j1/cmd_pos', 10)
        self.j2_pub = self.create_publisher(Float64, '/arm/j2/cmd_pos', 10)
        self.grab_pub = self.create_publisher(Bool, '/gripper/grab', 10)
        
        self.j1 = 0.0
        self.j2 = 0.0
        self.grab = False
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(msg)
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'w': self.j1 += 0.05
                elif key == 's': self.j1 -= 0.05
                elif key == 'a': self.j2 += 0.05
                elif key == 'd': self.j2 -= 0.05
                elif key == ' ': self.grab = not self.grab
                elif key == 'r': self.j1 = 0.0; self.j2 = 0.0
                elif key == 'q': 
                    print(f"\n✅ CALIBRATION DONE.\nFinal J1: {self.j1:.3f}\nFinal J2: {self.j2:.3f}\n")
                    break
                
                if key:
                    self.j1_pub.publish(Float64(data=self.j1))
                    self.j2_pub.publish(Float64(data=self.j2))
                    self.grab_pub.publish(Bool(data=self.grab))
                    print(f"\rJ1: {self.j1:.3f} | J2: {self.j2:.3f} | Grab: {self.grab}    ", end="")
                    
        except Exception as e:
            print(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    tuner = ArmLabTuner()
    tuner.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

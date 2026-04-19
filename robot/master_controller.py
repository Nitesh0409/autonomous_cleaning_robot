import subprocess
import sys, select, termios, tty

# Controller Display
msg = """
--- MASTER CALIBRATION HUB ---
Control the 'Calibration Cross' to verify Gazebo's World Scale.

[W / S] : Move X +/- 0.1m
[A / D] : Move Y +/- 0.1m
[E / Q] : Rotate Yaw +/- 5 deg
[Z / X] : Scale +/- 0.1
[R]     : Reset to (0,0)
[SPACE] : Print Alignment Report

CURRENT VALUES:
"""

class MasterController:
    def __init__(self):
        self.x = 1.0
        self.y = 0.0
        self.yaw = 0.0
        self.scale = 1.0
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def update_gazebo(self):
        # We use 'gz service' for instantaneous teleportation without a bridge
        cmd = f"gz service -s /world/master_calibration/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req 'name: \"calib_cross\", position: {{x: {self.x}, y: {self.y}, z: 0.1}}, orientation: {{z: {math.sin(math.radians(self.yaw/2))}, w: {math.cos(math.radians(self.yaw/2))}}}'"
        subprocess.run(['wsl', '-d', 'Ubuntu-24.04', 'bash', '-c', f"source /opt/ros/jazzy/setup.bash && {cmd}"], capture_output=True)

    def run(self):
        print(msg)
        try:
            while True:
                key = self.get_key()
                if key == 'w': self.x += 0.1
                elif key == 's': self.x -= 0.1
                elif key == 'a': self.y += 0.1
                elif key == 'd': self.y -= 0.1
                elif key == 'q': self.yaw += 5.0
                elif key == 'e': self.yaw -= 5.0
                elif key == 'z': self.scale += 0.1
                elif key == 'x': self.scale -= 0.1
                elif key == 'r': self.x = 0.0; self.y = 0.0; self.yaw = 0.0
                elif key == ' ': 
                    print(f"\n📏 ALIGNMENT REPORT:\nPos:({self.x:.2f}, {self.y:.2f})\nYaw:{self.yaw:.1f}\nScale:{self.scale:.2f}\n")
                elif key == '\x03': break # Ctrl+C
                
                if key:
                    self.update_gazebo()
                    print(f"\rX:{self.x:.2f} | Y:{self.y:.2f} | Yaw:{self.yaw:.1f} | Scale:{self.scale:.2f}   ", end="")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

import math
if __name__ == '__main__':
    ctrl = MasterController()
    ctrl.run()

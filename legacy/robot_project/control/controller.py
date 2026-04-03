import numpy as np

class RobotController:
    def __init__(self, kp_v=0.1, kp_a=0.01):
        """
        kp_v: Prop gain for linear velocity (m/s)
        kp_a: Prop gain for angular velocity (rad/s)
        """
        self.kp_v = kp_v
        self.kp_a = kp_a

    def compute_cmd(self, cur_pos, target_pos, cur_heading, force=(0,0)):
        """
        Based on current state and a planning force, compute (linear_v, angular_v)
        cur_pos: (x, y)
        target_pos: (x, y)
        cur_heading: angle in radians
        force: (fx, fy) from planning behavior
        """
        # If we have a force from planning (like potential field), use it to set desired heading
        if np.hypot(force[0], force[1]) > 0.01:
            desired_heading = np.atan2(force[1], force[0])
            error_heading = self.normalize_angle(desired_heading - cur_heading)
            angular_v = error_heading * self.kp_a
            
            # Simple linear velocity: proportional to force magnitude
            linear_v = np.hypot(force[0], force[1]) * self.kp_v
        else:
            # Traditional point-to-point if no force provided
            dx = target_pos[0] - cur_pos[0]
            dy = target_pos[1] - cur_pos[1]
            dist = np.hypot(dx, dy)
            desired_heading = np.atan2(dy, dx)
            error_heading = self.normalize_angle(desired_heading - cur_heading)
            
            angular_v = error_heading * self.kp_a
            linear_v = dist * self.kp_v if dist > 5 else 0.0

        # Clipping to robot limits
        linear_v = np.clip(linear_v, -1.0, 1.0)
        angular_v = np.clip(angular_v, -2.0, 2.0)

        return float(linear_v), float(angular_v)

    @staticmethod
    def normalize_angle(angle):
        while angle > np.pi: angle -= 2.0 * np.pi
        while angle < -np.pi: angle += 2.0 * np.pi
        return angle

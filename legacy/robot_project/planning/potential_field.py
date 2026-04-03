import numpy as np

class PotentialFieldPlanner:
    def __init__(self, k_att=1.0, k_rep=100.0, d_min=50.0):
        """
        k_att: Attractive force gain
        k_rep: Repulsive force gain
        d_min: Distance threshold for repulsion to start
        """
        self.k_att = k_att
        self.k_rep = k_rep
        self.d_min = d_min

    def compute_force(self, robot_pos, target_pos, obstacles):
        """
        robot_pos, target_pos: (x, y) coordinates
        obstacles: List of (x, y, radius) tuples
        Returns: (fx, fy) net force
        """
        # 1. Attractive force to target
        dist_target = np.hypot(target_pos[0] - robot_pos[0], target_pos[1] - robot_pos[1])
        fx = self.k_att * (target_pos[0] - robot_pos[0])
        fy = self.k_att * (target_pos[1] - robot_pos[1])

        # 2. Repulsive force from obstacles
        for ox, oy, r in obstacles:
            dx = robot_pos[0] - ox
            dy = robot_pos[1] - oy
            dist_obs = np.hypot(dx, dy) - r
            
            if dist_obs < self.d_min:
                if dist_obs < 0.1: dist_obs = 0.1 # Avoid division by zero
                # Magnitude of repulsion grows as we get closer
                rep_mag = self.k_rep * (1.0/dist_obs - 1.0/self.d_min) * (1.0/(dist_obs**2))
                fx += rep_mag * (dx / dist_obs)
                fy += rep_mag * (dy / dist_obs)

        return fx, fy

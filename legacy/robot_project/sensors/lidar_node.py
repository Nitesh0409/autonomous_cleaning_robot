import numpy as np

class LidarNode:
    def __init__(self, num_points=360, max_dist=1000):
        """
        Simulated LIDAR node for obstacle detection.
        num_points: Number of distance readings per 360 degrees
        max_dist: Maximum range in cm/mm
        """
        self.num_points = num_points
        self.max_dist = max_dist

    def get_readings(self, obstacles=[]):
        """
        Simulates LIDAR scan given obstacle coordinates.
        obstacles: List of (dist, angle) or (x, y) relative to robot
        """
        # For now, returns a random scan with some base range
        readings = self.max_dist * (0.8 + 0.2 * np.random.rand(self.num_points))
        return readings

    @staticmethod
    def detect_closest_obstacle(readings, threshold=100):
        """
        Checks for obstacles within the threshold range.
        Returns: True and (angle, dist) if found, else False, None
        """
        min_dist = np.min(readings)
        if min_dist < threshold:
            angle = np.argmin(readings) # Index as angle indicator
            return True, (angle, min_dist)
        return False, None

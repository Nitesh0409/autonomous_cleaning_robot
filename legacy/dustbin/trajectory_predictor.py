import numpy as np

class TrajectoryPredictor:
    def __init__(self, gravity=9.81):
        self.g = gravity
        self.points = [] # List of (time, x, y, z)
        self.min_points_to_predict = 5
        
    def add_observation(self, t, x, y, z):
        self.points.append((t, x, y, z))
        if len(self.points) > 20: # Keep only recent observations
            self.points.pop(0)

    def predict_landing(self):
        if len(self.points) < self.min_points_to_predict:
            return None
        
        # Extract data
        data = np.array(self.points)
        ts = data[:, 0] - data[0, 0] # Relative time
        xs = data[:, 1]
        ys = data[:, 2]
        zs = data[:, 3]

        # Fit X and Y (Linear velocity)
        # x(t) = vx * t + x0
        # y(t) = vy * t + y0
        vx_fit = np.polyfit(ts, xs, 1)
        vy_fit = np.polyfit(ts, ys, 1)
        
        # Fit Z (Parabolic motion: z(t) = -0.5*g*t^2 + vz0*t + z0)
        # However, we often have drag or measurement noise, so let's fit a general 2nd order poly
        z_fit = np.polyfit(ts, zs, 2)
        
        # z_fit[0] should be ~ -4.9 (0.5 * gravity)
        # Calculate when z(t) hits 0 (landing height)
        # poly is: a*t^2 + b*t + c = 0
        roots = np.roots(z_fit)
        
        # Find the positive root in the future
        future_roots = [r.real for r in roots if r.real > ts[-1]]
        if not future_roots:
            return None
        
        t_land = min(future_roots)
        
        # Predict X and Y at t_land
        x_land = np.polyval(vx_fit, t_land)
        y_land = np.polyval(vy_fit, t_land)
        
        return x_land, y_land, t_land + data[0, 0]

# Simple Unit Test Logic
if __name__ == "__main__":
    tp = TrajectoryPredictor()
    # Simulate a ball thrown from (0,0,1) with some velocity
    for i in range(10):
        t = i * 0.1
        x = 2.0 * t
        y = 0.5 * t
        z = 1.0 + 5.0 * t - 0.5 * 9.81 * t**2
        tp.add_observation(t, x, y, z)
    
    result = tp.predict_landing()
    if result:
        print(f"Predicted Landing: X={result[0]:.2f}, Y={result[1]:.2f} at T={result[2]:.2f}")

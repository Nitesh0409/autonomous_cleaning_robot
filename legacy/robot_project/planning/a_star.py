import heapq
import numpy as np

class AStarPlanner:
    def __init__(self, grid_size=10, obstacle_threshold=70):
        """
        grid_size: Size of local planning grid (e.g., 10px per cell)
        obstacle_threshold: Value in occupancy grid to treat as obstacle
        """
        self.grid_size = grid_size
        self.obstacle_threshold = obstacle_threshold

    def plan_path(self, start, goal, grid_map):
        """
        start, goal: (x, y) real-world coordinates
        grid_map: 2D numpy array representing occupancy
        Returns: List of (x, y) coordinates for the path
        """
        # 1. Convert to grid space
        sx, sy = int(start[0]/self.grid_size), int(start[1]/self.grid_size)
        gx, gy = int(goal[0]/self.grid_size), int(goal[1]/self.grid_size)
        
        # 2. Basic A*
        rows, cols = grid_map.shape
        open_list = [(0, (sx, sy))]
        came_from = {}
        g_score = {(sx, sy): 0}
        f_score = {(sx, sy): self.heuristic((sx, sy), (gx, gy))}

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == (gx, gy):
                return self.reconstruct_path(came_from, current)

            for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]: # 4-connectivity
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                    if grid_map[neighbor] > self.obstacle_threshold:
                        continue # Skip obstacles

                    tentative_g = g_score[current] + 1
                    if tentative_g < g_score.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self.heuristic(neighbor, (gx, gy))
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return None # No path found

    @staticmethod
    def heuristic(a, b):
        return np.hypot(a[0] - b[0], a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        # Convert back to real-world coordinates
        return [(p[0] * self.grid_size, p[1] * self.grid_size) for p in path]

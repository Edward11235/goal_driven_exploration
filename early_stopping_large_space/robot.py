import numpy as np
from path_planner_earlystop import plan_path

class Robot:
    def __init__(self, start, goal, map_size, vision_radius=50):
        self.current_position = start
        self.goal = goal
        self.vision_radius = vision_radius
        self.known_map = np.full((map_size, map_size), -1, dtype=int)
        self.known_map[start[1], start[0]] = 0  # Fix initialization
        self.visited_path = [start]

    def sense(self, actual_map):
        """Corrected coordinate handling"""
        x, y = self.current_position  # (column, row)
        vr = self.vision_radius
  
        row_min = max(0, y - vr)
        row_max = min(actual_map.shape[0]-1, y + vr)
        col_min = max(0, x - vr)
        col_max = min(actual_map.shape[1]-1, x + vr)
  
        self.known_map[row_min:row_max+1, col_min:col_max+1] = \
            actual_map[row_min:row_max+1, col_min:col_max+1]

    def plan_path(self):
        start = self.current_position
        goal = self.goal
        return plan_path(self.known_map, start, goal)

    def move_one_step(self, path):
        """Enhanced validation"""
        if not path or len(path) < 2:
            return
  
        next_x, next_y = path[1]
  
        if (0 <= next_x < self.known_map.shape[1] and
            0 <= next_y < self.known_map.shape[0] and
            self.known_map[next_y, next_x] == 0):
      
            self.current_position = (next_x, next_y)
            self.visited_path.append((next_x, next_y))
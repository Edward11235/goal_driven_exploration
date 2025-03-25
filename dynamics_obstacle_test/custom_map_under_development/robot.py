
import path_planner_astar
import time
import numpy as np
from scipy.ndimage import label, center_of_mass

planner_call_count = 0
total_planner_time = 0.0
print_average_time = True

class Robot:
    def __init__(self, maze, start=(0,0), goal=None, vision_radius=5, algorithm='astar'):
        self.maze = maze
        self.pos = start
        self.goal = goal if goal is not None else (maze.height-1, maze.width-1)
        self.vision_radius = vision_radius
        self.known_map = [[-1 for _ in range(maze.width)] for _ in range(maze.height)]
        self.visited_path = [self.pos]
        self.dynamic_obstacles = {}  # id: (current_pos, prev_pos)
        
        sx, sy = self.pos
        self.known_map[sx][sy] = 0
        self.sense(self.pos)
        self._planner_func = path_planner_astar.astar

    def sense(self, prev_robot_pos):
        x, y = self.pos
        r = self.vision_radius

        obstacle_grid = np.zeros((self.maze.height, self.maze.width), dtype=int)
        for i in range(max(0, x - r), min(self.maze.height, x + r + 1)):
            for j in range(max(0, y - r), min(self.maze.width, y + r + 1)):
                if abs(i - x) + abs(j - y) <= r:
                    if self.maze.grid[i][j] == 1:
                        obstacle_grid[i, j] = 1
                        self.known_map[i][j] = 1
                    else:
                        self.known_map[i][j] = 0

        labeled, num_features = label(obstacle_grid)
        centroids = center_of_mass(obstacle_grid, labeled, range(1, num_features + 1))

        robot_move_offset = np.array(self.pos) - np.array(prev_robot_pos)

        new_dynamic_obstacles = {}
        for idx, centroid in enumerate(centroids):
            cy, cx = map(int, centroid)
            matched_id = None
            min_dist = float('inf')
            
            # Adjust previous obstacle positions based on robot movement
            for oid, (prev_obstacle_pos, _) in self.dynamic_obstacles.items():
                adjusted_prev_pos = np.array(prev_obstacle_pos) + robot_move_offset
                dist = np.hypot(cx - adjusted_prev_pos[0], cy - adjusted_prev_pos[1])
                
                if dist < min_dist and dist <= 5:
                    min_dist = dist
                    matched_id = oid
            
            if matched_id is not None:
                prev_pos = self.dynamic_obstacles[matched_id][0]
                new_dynamic_obstacles[matched_id] = ((cx, cy), prev_pos)
            else:
                new_dynamic_obstacles[idx] = ((cx, cy), (cx, cy))

        self.dynamic_obstacles = new_dynamic_obstacles


    def predict_dynamic_obstacle_positions(self):
        predictions = []
        for current_pos, prev_pos in self.dynamic_obstacles.values():
            velocity = np.array(current_pos) - np.array(prev_pos)
            predicted_pos = np.array(current_pos) + velocity
            px, py = predicted_pos.astype(int)
            if 0 <= px < self.maze.width and 0 <= py < self.maze.height:
                predictions.append((px, py))
        return predictions

    def plan_path(self):
        global planner_call_count, total_planner_time

        temp_grid = [row.copy() for row in self.known_map]
        predicted_positions = self.predict_dynamic_obstacle_positions()
        for x, y in predicted_positions:
            if temp_grid[x][y] == 0:
                temp_grid[x][y] = 1

        start_time = time.time()
        path = self._planner_func(self.pos, self.goal, temp_grid)
        elapsed_time = time.time() - start_time

        planner_call_count += 1
        total_planner_time += elapsed_time

        if print_average_time:
            print(f"Avg planning time: {total_planner_time/planner_call_count:.4f}s")

        return path

    def move_one_step(self, path):
        if path is None or len(path) < 2:
            return False
        prev_pos = self.pos  # Store current pos as previous
        next_cell = path[1]
        self.pos = next_cell
        self.visited_path.append(self.pos)
        self.sense(prev_pos)  # Call sense with previous robot position
        return True

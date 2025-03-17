# robot.py (modified to use D* Lite)
"""
Robot module: navigates maze using D* Lite algorithm.
"""

from path_planner import DStarLite

class Robot:
    def __init__(self, maze, start=(0,0), goal=None, vision_radius=5):
        self.maze = maze
        self.pos = start
        self.goal = goal if goal is not None else (maze.height-1, maze.width-1)
        self.vision_radius = vision_radius
        self.known_map = [[-1 for _ in range(maze.width)] for _ in range(maze.height)]
        sx, sy = self.pos
        self.known_map[sx][sy] = 0
        self.visited_path = [self.pos]
        self.planner = DStarLite(start=self.pos, goal=self.goal, grid=self.known_map)
        self.sense()
  
    def sense(self):
        changed_cells = []
        x, y = self.pos
        r = self.vision_radius
        for i in range(max(0, x-r), min(self.maze.height, x+r+1)):
            for j in range(max(0, y-r), min(self.maze.width, y+r+1)):
                if abs(i-x) + abs(j-y) <= r:
                    old_val = self.known_map[i][j]
                    new_val = 1 if self.maze.grid[i][j] == 1 else 0
                    if old_val != new_val:
                        changed_cells.append((i, j))
                        self.known_map[i][j] = new_val
        if changed_cells:
            self.planner.handle_changed_edges(changed_cells)
  
    def plan_path(self):
        self.planner.update_start(self.pos)
        self.planner.compute_shortest_path()
        return self.planner.plan_path()
  
    def move_one_step(self, path):
        if not path or len(path) < 2:
            return False
          
        next_cell = path[1]
        # Check actual maze for obstacles
        if self.maze.is_wall(next_cell):
            x, y = next_cell
            self.known_map[x][y] = 1
            self.planner.handle_changed_edges([next_cell])
            return False
          
        self.pos = next_cell
        self.visited_path.append(self.pos)
        self.sense()
        return True
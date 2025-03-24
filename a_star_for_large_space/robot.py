"""
Robot module: contains Robot class that navigates the maze with limited vision.
"""
import path_planner_astar
import path_planner_jps
import time

planner_call_count = 0
total_planner_time = 0.0
print_average_time = True

def get_average_planner_time():
    global planner_call_count, total_planner_time
    if planner_call_count == 0:
        return 0
    return total_planner_time / planner_call_count

class Robot:
    def __init__(self, maze, start=(0,0), goal=None, vision_radius=5, algorithm='astar'):
        """
        Initialize the robot in a given maze.
        - maze: Maze object (actual environment).
        - start: starting position (x, y), default (0,0).
        - goal: target position (x, y). If None, defaults to bottom-right corner.
        - vision_radius: how far the robot can see (in Manhattan distance).
        - algorithm: pathfinding algorithm to use ('astar' or 'jps').
        """
        self.maze = maze
        self.pos = start
        self.goal = goal if goal is not None else (maze.height-1, maze.width-1)
        self.vision_radius = vision_radius
        # Robot's knowledge of the maze grid: -1 = unknown, 0 = free, 1 = wall
        self.known_map = [[-1 for _ in range(maze.width)] for _ in range(maze.height)]
        # Mark starting cell as known free (robot starts there)
        sx, sy = self.pos
        self.known_map[sx][sy] = 0
        # Record the path taken by the robot (visited cells)
        self.visited_path = [self.pos]
        # Initial sensor sweep around start
        self.sense()
        # Select path planning function based on the specified algorithm
        if algorithm == 'astar':
            self._planner_func = path_planner_astar.astar
        elif algorithm == 'jps':
            self._planner_func = path_planner_jps.jps
        else:
            raise ValueError("Unknown algorithm specified. Choose 'astar' or 'jps'.")

    def sense(self):
        """Sense the surrounding area within vision_radius and update known_map."""
        x, y = self.pos
        r = self.vision_radius
        for i in range(max(0, x-r), min(self.maze.height, x+r+1)):
            for j in range(max(0, y-r), min(self.maze.width, y+r+1)):
                # Check Manhattan distance to ensure within square's radius (diamond shape visibility)
                if abs(i - x) + abs(j - y) <= r:
                    # Reveal actual cell: mark walls and free spaces in known_map
                    if self.maze.grid[i][j] == 1:
                        self.known_map[i][j] = 1  # wall
                    else:
                        self.known_map[i][j] = 0  # free space

    def plan_path(self):
        """
        Plan a path from the robot's current position to the goal using known information.
        Returns a list of cells from current position to goal (inclusive), or None if no path is found.
        """
        global planner_call_count, total_planner_time

        start_time = time.time()
        path = self._planner_func(self.pos, self.goal, self.known_map)
        elapsed_time = time.time() - start_time

        planner_call_count += 1
        total_planner_time += elapsed_time

        if print_average_time:
            avg = get_average_planner_time()
            print(avg)

        return path

    def move_one_step(self, path):
        """
        Move the robot one step along the given path.
        Path is a list of cells from current position to goal. Move to the next cell on the path.
        Returns True if a move was made, or False if there's no move (e.g., at goal or no path).
        """
        if path is None or len(path) < 2:
            # No path given, or we're already at the goal
            return False
        # Next position is the second cell in the path (first is current position)
        next_cell = path[1]
        # Update robot position
        self.pos = next_cell
        # Record this move in the visited path
        self.visited_path.append(self.pos)
        # Sense environment from the new position
        self.sense()
        return True

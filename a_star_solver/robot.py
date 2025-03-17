"""
Robot module: contains Robot class that navigates the maze with limited vision.
"""
from path_planner import astar

class Robot:
    def __init__(self, maze, start=(0,0), goal=None, vision_radius=5):
        """
        Initialize the robot in a given maze.
        - maze: Maze object (actual environment).
        - start: starting position (x, y), default (0,0).
        - goal: target position (x, y). If None, defaults to bottom-right corner.
        - vision_radius: how far the robot can see (in Manhattan distance).
        """
        self.maze = maze
        self.pos = start
        self.goal = goal if goal is not None else (maze.height-1, maze.width-1)
        self.vision_radius = vision_radius
        # Robot's knowledge of the maze: -1 = unknown, 0 = free, 1 = wall.
        self.known_map = [[-1 for _ in range(maze.width)] for _ in range(maze.height)]
        # Mark starting cell as known free (robot is there)
        sx, sy = self.pos
        self.known_map[sx][sy] = 0
        # Track the path taken by the robot (visited cells)
        self.visited_path = [self.pos]
        # Sense initial surroundings
        self.sense()
    
    def sense(self):
        """Sense the surrounding area within vision_radius and update known_map."""
        x, y = self.pos
        r = self.vision_radius
        for i in range(max(0, x-r), min(self.maze.height, x+r+1)):
            for j in range(max(0, y-r), min(self.maze.width, y+r+1)):
                # Check Manhattan distance for visibility
                if abs(i - x) + abs(j - y) <= r:
                    # Reveal the actual cell from the maze
                    if self.maze.grid[i][j] == 1:
                        self.known_map[i][j] = 1  # wall
                    else:
                        self.known_map[i][j] = 0  # free space
    
    def plan_path(self):
        """
        Plan a path from the robot's current position to the goal using known information.
        Returns a list of cells from current position to goal (inclusive), or None if no path is found.
        """
        return astar(self.pos, self.goal, self.known_map)
    
    def move_one_step(self, path):
        """
        Move the robot one step along the given path.
        Path is a list of cells from current position to goal. Move to the next cell on the path.
        Returns True if a move was made, or False if there's no move (e.g., at goal or no path).
        """
        if path is None or len(path) < 2:
            # No path or already at goal
            return False
        # Next position is the second cell in the path (first is current position)
        next_cell = path[1]
        # Update robot position
        self.pos = next_cell
        # Record the move in visited path
        self.visited_path.append(self.pos)
        # Sense from the new position to update knowledge of the map
        self.sense()
        return True

"""
Maze generation module: contains Maze class for generating a random maze.
"""
import random
from collections import deque

class Maze:
    def __init__(self, width=20, height=20):
        """
        Initialize a maze of given width and height and generate random walls.
        Ensures that a path from the top-left to bottom-right exists.
        """
        self.width = width
        self.height = height
        # Grid representation: 0 = free space, 1 = wall
        self.grid = [[0 for _ in range(width)] for _ in range(height)]
        self._generate_random_maze()
    
    def _generate_random_maze(self):
        """Generate random walls in the maze, ensuring start and goal are reachable."""
        # Step 1: Create a random path from start (0,0) to goal (height-1, width-1)
        path = []
        x, y = 0, 0
        target_x, target_y = self.height - 1, self.width - 1
        # List of moves required to reach bottom-right: (target_x) downs and (target_y) rights
        moves = [(1, 0)] * target_x + [(0, 1)] * target_y
        random.shuffle(moves)  # Randomize the order of moves to create a unique path
        path.append((x, y))
        for dx, dy in moves:
            x += dx
            y += dy
            path.append((x, y))
        path_set = set(path)  # Use a set for quick lookup of path cells

        # Step 2: Fill the grid with random walls/free spaces, but keep path cells free
        for i in range(self.height):
            for j in range(self.width):
                if (i, j) in path_set:
                    # Ensure path cells (including start & goal) are free
                    self.grid[i][j] = 0
                else:
                    # Randomly assign obstacles to non-path cells (30% chance wall)
                    self.grid[i][j] = 1 if random.random() < 0.3 else 0

    def _is_reachable(self, start, goal):
        """(Optional utility) Check via BFS if there is a path from start to goal ignoring walls."""
        sx, sy = start
        gx, gy = goal
        if self.grid[sx][sy] != 0 or self.grid[gx][gy] != 0:
            return False
        visited = [[False]*self.width for _ in range(self.height)]
        q = deque([start])
        visited[sx][sy] = True
        while q:
            x, y = q.popleft()
            if (x, y) == goal:
                return True
            for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.height and 0 <= ny < self.width:
                    if not visited[nx][ny] and self.grid[nx][ny] == 0:
                        visited[nx][ny] = True
                        q.append((nx, ny))
        return False

    def is_wall(self, cell):
        """Return True if the given cell (x,y) is a wall in the maze."""
        x, y = cell
        return self.grid[x][y] == 1

    def is_free(self, cell):
        """Return True if the given cell (x,y) is free (no wall) in the maze."""
        x, y = cell
        return self.grid[x][y] == 0

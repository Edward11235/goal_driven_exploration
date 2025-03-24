"""
Maze generation module: contains Maze class for generating a random 20x20 maze.
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
        while True:
            # Randomly assign walls to each cell (excluding start and goal)
            for i in range(self.height):
                for j in range(self.width):
                    if (i == 0 and j == 0) or (i == self.height-1 and j == self.width-1):
                        # Ensure start and goal cells are free
                        self.grid[i][j] = 0
                    else:
                        # 30% chance of wall, 70% chance of free space
                        self.grid[i][j] = 1 if random.random() < 0.3 else 0
                        
            # Check if start->goal path exists in this generated maze
            if self._is_reachable((0,0), (self.height-1, self.width-1)):
                break
    
    def _is_reachable(self, start, goal):
        """Check if there is a path from start to goal (BFS) ignoring walls."""
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
            # Explore neighbors (4-directional)
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
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

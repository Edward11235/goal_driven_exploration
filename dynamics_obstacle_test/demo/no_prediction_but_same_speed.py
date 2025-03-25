import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import heapq
from scipy.ndimage import label, center_of_mass

GRID_SIZE = 50
VISION_RADIUS = 25
BOX_SIZE = 3

def astar(start, goal, grid):
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current:
                path.append(current)
                current = came_from.get(current)
            return path[::-1]

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1, -1), (-1, 1), (1, -1), (1,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if 0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE:
                if grid[neighbor[1], neighbor[0]] == 1:
                    continue
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    heapq.heappush(open_set, (tentative_g + heuristic(neighbor, goal), neighbor))
    return None

class Robot:
    def __init__(self, pos, goal):
        self.pos = pos
        self.goal = goal
        self.known_map = -np.ones((GRID_SIZE, GRID_SIZE), dtype=int)

    def sense(self, grid):
        x, y = self.pos
        obstacle_grid = np.zeros_like(grid)
        for i in range(max(0, x-VISION_RADIUS), min(GRID_SIZE, x+VISION_RADIUS+1)):
            for j in range(max(0, y-VISION_RADIUS), min(GRID_SIZE, y+VISION_RADIUS+1)):
                if abs(i - x) + abs(j - y) <= VISION_RADIUS:
                    obstacle_grid[j, i] = grid[j, i]
                    self.known_map[j, i] = grid[j, i]

        labeled, num = label(obstacle_grid)
        centroids = center_of_mass(obstacle_grid, labeled, range(1, num+1))
        obstacles = [(int(c[1]), int(c[0])) for c in centroids]
        return obstacles

    def plan(self):
        return astar(self.pos, self.goal, self.known_map)

    def move(self, path):
        if path and len(path) > 1:
            self.pos = path[1]

# Initialize visualization
fig, ax = plt.subplots(figsize=(7,7))
plt.ion()
cmap = ListedColormap(['white', 'black', 'blue', 'green', 'red'])

# Initialize robot and obstacle
robot = Robot((0,0), (49,49))
goal_switch = {(0,0): (49,49), (49,49): (0,0)}
obstacle_horizontal = 15
obstacle_vertical = 30
obstacle_pos = (obstacle_horizontal, obstacle_vertical)
velocity = -1  # 1 = down, -1 = up

for step in range(1000):
    # Update obstacle position
    grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
    oy = obstacle_pos[1] + velocity
  
    # Reverse velocity at boundaries
    if oy >= 49 or oy <= 0:
        velocity *= -1
        oy = obstacle_pos[1] + velocity
  
    # Create 5x5 obstacle
    obstacle_pos = (obstacle_horizontal, oy)
    for dx in range(-BOX_SIZE, BOX_SIZE+1):
        for dy in range(-BOX_SIZE, BOX_SIZE+1):
            x, y = obstacle_pos[0]+dx, obstacle_pos[1]+dy
            if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
                grid[y, x] = 1

    
    # Robot operations
    robot.sense(grid)
    path = robot.plan()
    if path:
        robot.move(path)
    else:
        print("No path found!")
        # Check collision
        rx, ry = robot.pos
        if (obstacle_horizontal-BOX_SIZE <= rx <= obstacle_horizontal+BOX_SIZE) and (obstacle_pos[1]-BOX_SIZE <= ry <= obstacle_pos[1]+BOX_SIZE):
            ax.text(GRID_SIZE//2, GRID_SIZE//2, 'COLLIDED', 
                fontsize=20, color='red', ha='center', va='center')
            plt.pause(2)
        break

    

    # Update visualization
    vis_grid = grid.copy()
    vis_grid[robot.pos[1], robot.pos[0]] = 2
    vis_grid[robot.goal[1], robot.goal[0]] = 3

    ax.clear()
    ax.set_title(f"Step {step}")
    ax.imshow(vis_grid, cmap=cmap, origin='upper')
  
    # Switch goal when reached
    if robot.pos == robot.goal:
        robot.goal = goal_switch[robot.goal]
  
    plt.pause(0.1)

plt.ioff()
plt.show()
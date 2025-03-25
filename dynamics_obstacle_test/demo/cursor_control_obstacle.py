# Corrected and clearly formatted standalone demo script

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import heapq
from scipy.ndimage import label, center_of_mass

GRID_SIZE = 50
VISION_RADIUS = 10

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

fig, ax = plt.subplots(figsize=(7,7))
plt.ion()

robot = Robot((0,0), (49,49))
goal_switch = {(0,0):(49,49), (49,49):(0,0)}
cursor_pos = [25,25]

def on_mouse_move(event):
    if event.xdata is not None and event.ydata is not None:
        cursor_pos[0] = int(event.xdata)
        cursor_pos[1] = int(event.ydata)

fig.canvas.mpl_connect('motion_notify_event', on_mouse_move)
cmap = ListedColormap(['white', 'black', 'blue', 'green', 'red'])

for step in range(1000):
    grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)

    ox, oy = cursor_pos
    for dx in range(-2, 3):
        for dy in range(-2, 3):
            x, y = ox+dx, oy+dy
            if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
                grid[y,x] = 1

    robot.sense(grid)
    path = robot.plan()
    robot.move(path)

    if robot.pos == robot.goal:
        robot.goal = goal_switch[robot.goal]

    vis_grid = grid.copy()
    vis_grid[robot.pos[1], robot.pos[0]] = 2
    vis_grid[robot.goal[1], robot.goal[0]] = 3

    ax.clear()
    ax.set_title(f"Step {step}")
    ax.imshow(vis_grid, cmap=cmap, origin='upper')
    plt.pause(0.1)

plt.ioff()
plt.show()


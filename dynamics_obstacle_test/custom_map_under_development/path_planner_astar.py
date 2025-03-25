"""
Updated A* with dynamic obstacle support
"""
import heapq
import math
from itertools import product

early_stop_distance = 50
OBSTACLE_WEIGHT = 0.5
DISTANCE_THRESHOLD = 3

_penalty_cache = {}
for dx, dy in product(range(-DISTANCE_THRESHOLD, DISTANCE_THRESHOLD+1), repeat=2):
    distance = abs(dx) + abs(dy)
    if 0 < distance <= DISTANCE_THRESHOLD:
        _penalty_cache[(dx, dy)] = OBSTACLE_WEIGHT / (distance ** 1.5)

def astar(start, goal, grid):
    height = len(grid)
    width = len(grid[0]) if height > 0 else 0

    def heuristic(cell, goal):
        return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

    def get_obstacle_penalty(x, y):
        penalty = 0.0
        min_x = max(0, x - DISTANCE_THRESHOLD)
        max_x = min(height-1, x + DISTANCE_THRESHOLD)
        min_y = max(0, y - DISTANCE_THRESHOLD)
        max_y = min(width-1, y + DISTANCE_THRESHOLD)
    
        for i in range(min_x, max_x+1):
            for j in range(min_y, max_y+1):
                if grid[i][j] in [1, 2]:  # Consider both static and dynamic obstacles
                    dx = i - x
                    dy = j - y
                    penalty += _penalty_cache.get((dx, dy), 0)
        return penalty

    
    open_heap = []
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    heapq.heappush(open_heap, (f_score[start], g_score[start], start[0], start[1], None))
    closed_set = set()

    while open_heap:
        f, g, cx, cy, parent = heapq.heappop(open_heap)
        current = (cx, cy)
      
        if current in closed_set and g > g_score.get(current, float('inf')):
            continue
          
        if parent is not None:
            came_from[current] = parent

        distance = math.sqrt((current[0] - start[0])**2 + (current[1] - start[1])**2)
        if current == goal or distance > early_stop_distance:
            path = []
            node = current
            while node is not None:
                path.append(node)
                node = came_from.get(node)
            path.reverse()
            return path

        closed_set.add(current)

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or nx >= height or ny < 0 or ny >= width or grid[nx][ny] == 1:
                continue

            obstacle_penalty = get_obstacle_penalty(nx, ny)
            tentative_g = g + 1 + obstacle_penalty

            if tentative_g < g_score.get((nx, ny), float('inf')):
                g_score[(nx, ny)] = tentative_g
                f_score[(nx, ny)] = tentative_g + heuristic((nx, ny), goal)
                heapq.heappush(open_heap, (f_score[(nx, ny)], tentative_g, nx, ny, current))

    return None
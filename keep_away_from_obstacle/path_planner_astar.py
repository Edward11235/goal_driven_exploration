"""
Path planning module (A*): Optimized A* with obstacle proximity costs
"""
import heapq
import math

early_stop_distance = 100
OBSTACLE_WEIGHT = 0.3  # Controls how strongly paths avoid obstacles

def astar(start, goal, grid):
    """
    Compute path from start to goal with obstacle avoidance tendency
    """

    dirs = ((-1, 0), (1, 0), (0, -1), (0, 1))
    
    def heuristic(cell, goal):
        return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

    def get_obstacle_penalty(x, y):
        """Optimized penalty calculation with boundary checks"""
        penalty = 0
        # Pre-check valid ranges
        valid_left = x > 0
        valid_right = x < height - 1
        valid_up = y > 0
        valid_down = y < width - 1
      
        for dx, dy in dirs:
            # Check boundaries using pre-computed validity
            if ((dx == -1 and not valid_left) or
                (dx == 1 and not valid_right) or
                (dy == -1 and not valid_up) or
                (dy == 1 and not valid_down)):
                continue
              
            if grid[x + dx][y + dy] == 1:
                penalty += OBSTACLE_WEIGHT / (dx**2 + dy**2 + 1)
        return penalty

    height = len(grid)
    width = len(grid[0]) if height > 0 else 0

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

        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
            nx, ny = cx + dx, cy + dy
            neighbor = (nx, ny)
          
            if nx < 0 or nx >= height or ny < 0 or ny >= width or grid[nx][ny] == 1:
                continue

            # Modified cost calculation with obstacle penalty
            obstacle_penalty = get_obstacle_penalty(nx, ny)
            tentative_g = g + 1 + obstacle_penalty

            if tentative_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f_score[neighbor], tentative_g, nx, ny, current))

    return None
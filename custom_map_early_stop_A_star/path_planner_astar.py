"""
Path planning module (A*): Optimized A* algorithm using a priority queue (heap).
"""
import heapq
import math

early_stop_distance = 100

def astar(start, goal, grid):
    """
    Compute shortest path from start to goal on a grid using A* with a binary heap.
    - start, goal: tuple (x, y) coordinates on the grid.
    - grid: 2D list (or similar) where 0 = free, 1 = wall, -1 = unknown (treated as free for planning).
    Returns: list of (x, y) cells from start to goal if path found, else None.
    """
    # Manhattan distance heuristic (grid is uniform cost, no diagonal moves considered)
    def heuristic(cell, goal):
        return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])
    height = len(grid)
    width = len(grid[0]) if height > 0 else 0

    # Priority queue for open set: stores (f_score, g_score, x, y, parent_coord)
    open_heap = []
    # Dictionaries for best distances and path reconstruction
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    # Push start node into heap
    heapq.heappush(open_heap, (f_score[start], g_score[start], start[0], start[1], None))
    # Set to keep track of visited nodes (closed set)
    closed_set = set()

    while open_heap:
        # Pop the cell with lowest f_score
        f, g, cx, cy, parent = heapq.heappop(open_heap)
        current = (cx, cy)
        # If this node is already closed with a better score, skip it
        if current in closed_set and g > g_score.get(current, float('inf')):
            continue
        # Record parent for path reconstruction
        if parent is not None:
            came_from[current] = parent
        # Check if goal reached

        distance = math.sqrt((current[0] - start[0])**2 + (current[1] - start[1])**2)

        if current == goal or distance > early_stop_distance:
            # Reconstruct path by backtracking parents
            path = []
            node = current
            while node is not None:
                path.append(node)
                node = came_from.get(node)
            path.reverse()
            return path
        # Mark current node as processed
        closed_set.add(current)
        # Explore 4-directional neighbors
        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
            nx, ny = cx + dx, cy + dy
            neighbor = (nx, ny)
            # Skip out-of-bounds or known wall neighbors
            if nx < 0 or nx >= height or ny < 0 or ny >= width or grid[nx][ny] == 1:
                continue
            # Unknown (-1) or free (0) cells are traversable
            tentative_g = g + 1
            if tentative_g < g_score.get(neighbor, float('inf')):
                # Found a better path to neighbor
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f_score[neighbor], tentative_g, nx, ny, current))
    # No path found
    return None

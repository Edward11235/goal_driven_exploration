"""
Path planning module: contains A* algorithm for pathfinding on a grid.
"""

def astar(start, goal, grid):
    """
    Compute shortest path from start to goal on a grid using A*.
    - start, goal: tuple (x, y) coordinates on the grid.
    - grid: 2D list where 0 = free, 1 = wall, -1 = unknown (treated as free for planning).
    Returns: list of (x, y) cells from start to goal if path found, else None.
    """
    # Manhattan distance heuristic
    def heuristic(cell, goal):
        return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])
    
    height = len(grid)
    width = len(grid[0]) if height > 0 else 0
    
    # Open set: stores tuples (f_score, g_score, x, y, parent)
    open_list = []
    # Dictionaries to track best scores and parent links
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    open_list.append((f_score[start], g_score[start], start[0], start[1], None))
    
    # A* search loop
    while open_list:
        # Pop the cell with lowest f_score
        open_list.sort(key=lambda x: x[0])
        f, g, cx, cy, parent = open_list.pop(0)
        current = (cx, cy)
        # Skip if this path is not the best for this cell
        if g > g_score.get(current, float('inf')):
            continue
        # Record parent to reconstruct path
        if parent is not None:
            came_from[current] = parent
        # Goal reached
        if current == goal:
            # Reconstruct path by following parent links
            path = []
            node = current
            while node is not None:
                path.append(node)
                node = came_from.get(node)
            return path[::-1]  # reverse to get path from start to goal
        # Explore neighbors (4-directional moves)
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = cx + dx, cy + dy
            neighbor = (nx, ny)
            # Check bounds
            if nx < 0 or nx >= height or ny < 0 or ny >= width:
                continue
            # Skip if neighbor is a known wall
            if grid[nx][ny] == 1:
                continue
            # Unknown (-1) or free (0) cells are traversable
            tentative_g = g + 1
            if tentative_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                open_list.append((f_score[neighbor], tentative_g, nx, ny, current))
    # No path found
    return None

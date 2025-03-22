"""
Path planning module (Jump Point Search): Implements JPS for faster pathfinding on a grid.
"""
import heapq

def jps(start, goal, grid):
    """
    Compute shortest path from start to goal on a grid using Jump Point Search.
    - start, goal: tuple (x, y) coordinates on the grid.
    - grid: 2D list (or similar) where 0 = free, 1 = wall, -1 = unknown (unknown treated as free for planning).
    Returns: list of (x, y) cells from start to goal if path found, else None.
    """
    def heuristic(cell, goal):
        return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])
    height = len(grid)
    width = len(grid[0]) if height > 0 else 0

    # Open set as a priority queue with entries (f_score, g_score, x, y, parent)
    open_heap = []
    heapq.heappush(open_heap, (heuristic(start, goal), 0, start[0], start[1], None))
    came_from = {}
    g_score = {start: 0}
    closed_set = set()

    def jump(current, direction):
        """
        Jump from the current cell in the given direction until a jump point is found or path ends.
        Returns the coordinate of the jump point or None if none found in this direction.
        """
        dx, dy = direction
        x, y = current
        moved = False
        while True:
            x += dx; y += dy
            # Stop if out of bounds or hit a wall
            if x < 0 or x >= height or y < 0 or y >= width or grid[x][y] == 1:
                # If we haven't moved at all (immediate wall/boundary), no jump point
                return None if not moved else current
            moved = True
            current = (x, y)
            if current == goal:
                return current  # Goal reached by jumping
            # Check for forced neighbors (cells that create a necessary turn)
            if dx == 0:  # moving horizontally
                # If moving right (dy positive) or left (dy negative),
                # check cells above and below for openings next to obstacles
                if (x + 1 < height and grid[x+1][y] == 0 and grid[x+1][y - dy] == 1) or \
                   (x - 1 >= 0 and grid[x-1][y] == 0 and grid[x-1][y - dy] == 1):
                    return current  # Found a forced neighbor -> jump point
            elif dy == 0:  # moving vertically
                # If moving down (dx positive) or up (dx negative),
                # check cells to the left and right for openings next to obstacles
                if (y + 1 < width and grid[x][y+1] == 0 and grid[x - dx][y+1] == 1) or \
                   (y - 1 >= 0 and grid[x][y-1] == 0 and grid[x - dx][y-1] == 1):
                    return current  # Found a forced neighbor -> jump point
            # Continue jumping in the same direction (no forced neighbor yet)
            # (Diagonal moves would include additional checks, but in this grid we only move vertically or horizontally.)
    
    while open_heap:
        f, g, cx, cy, parent = heapq.heappop(open_heap)
        current = (cx, cy)
        # If this node is already closed with a better g_score, skip
        if current in closed_set and g > g_score.get(current, float('inf')):
            continue
        if parent is not None:
            came_from[current] = parent
        if current == goal:
            # Reconstruct jump point path by backtracking through parents
            path = []
            node = current
            while node is not None:
                path.append(node)
                node = came_from.get(node)
            path.reverse()
            # Expand intermediate cells between each jump point for a complete step-by-step path
            full_path = [path[0]]
            for i in range(1, len(path)):
                px, py = path[i-1]
                cx, cy = path[i]
                # Determine step direction from previous jump point to current jump point
                dx = 0 if cx == px else (1 if cx > px else -1)
                dy = 0 if cy == py else (1 if cy > py else -1)
                x, y = px, py
                # Move from previous jump point towards current jump point, adding intermediate cells
                while (x, y) != (cx, cy):
                    x += dx; y += dy
                    full_path.append((x, y))
            return full_path  # Full path from start to goal
        closed_set.add(current)

        # Determine which directions to explore from current
        directions = []
        if parent is None:
            # At start, consider all four cardinal directions
            directions = [(1,0), (-1,0), (0,1), (0,-1)]
            back_dir = None
        else:
            # Determine direction from parent to current
            px, py = parent
            dx = 0 if cx == px else (1 if cx > px else -1)
            dy = 0 if cy == py else (1 if cy > py else -1)
            # Always continue in the direction of travel
            directions.append((dx, dy))
            # Also consider perpendicular directions (potential forced turns)
            if dx == 0:  # current move was horizontal
                directions.extend([(1, 0), (-1, 0)])
            elif dy == 0:  # current move was vertical
                directions.extend([(0, 1), (0, -1)])
            # The direction directly back to the parent is not considered (to prevent immediate backtracking)
            back_dir = (-dx, -dy)
        # Explore in each allowed direction
        for dx, dy in directions:
            if back_dir and (dx, dy) == back_dir:
                continue  # Skip the direction leading back to parent
            jump_point = jump((cx, cy), (dx, dy))
            if jump_point is not None:
                jx, jy = jump_point
                # Compute new g_score to this jump point (Manhattan distance from current)
                new_g = g_score[current] + abs(jx - cx) + abs(jy - cy)
                if new_g < g_score.get(jump_point, float('inf')):
                    g_score[jump_point] = new_g
                    priority = new_g + heuristic(jump_point, goal)
                    heapq.heappush(open_heap, (priority, new_g, jx, jy, current))
    # No path found
    return None

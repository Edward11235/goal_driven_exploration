from heapq import heappush, heappop

def plan_path(grid, start, goal, block_size=100, sw=55, slim=60):
    rows, cols = len(grid), len(grid[0])
    def manhattan(x1, y1, x2, y2): return abs(x1-x2) + abs(y1-y2)
    sx, sy = start
    gx, gy = goal
    path = []
    current_start = start
    visited_starts = set()
    first_segment = True

    while True:
        sx, sy = current_start
        if (sx, sy) in visited_starts:
            break
        visited_starts.add((sx, sy))

        # Direct A* if within slim distance
        if manhattan(sx, sy, gx, gy) <= slim:
            open_list = [(manhattan(sx, sy, gx, gy), 0, sx, sy)]
            came_from = {(sx, sy): None}
            g_cost = {(sx, sy): 0}
            found = False
            while open_list:
                _, g, x, y = heappop(open_list)
                if (x, y) == (gx, gy):
                    found = True
                    break
                if g_cost[(x, y)] < g:
                    continue
                for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                    nx, ny = x+dx, y+dy
                    if 0 <= nx < cols and 0 <= ny < rows and grid[ny][nx] == 0:
                        ng = g + 1
                        if (nx, ny) not in g_cost or ng < g_cost[(nx, ny)]:
                            g_cost[(nx, ny)] = ng
                            came_from[(nx, ny)] = (x, y)
                            heappush(open_list, (ng + manhattan(nx, ny, gx, gy), ng, nx, ny))
            if not found:
                break
            seg_path = []
            node = (gx, gy)
            while node is not None:
                seg_path.append(node)
                node = came_from[node]
            seg_path.reverse()
            path.extend(seg_path if first_segment else seg_path[1:])
            return path

        # Sliding window search area
        wx0 = max(0, sx - block_size//2)
        wy0 = max(0, sy - block_size//2)
        wx1 = min(cols-1, wx0 + block_size - 1)
        wy1 = min(rows-1, wy0 + block_size - 1)
      
        H0 = manhattan(sx, sy, gx, gy)
        open_list = [(H0, 0, sx, sy)]
        came_from = {(sx, sy): None}
        g_cost = {(sx, sy): 0}
        best_node = (sx, sy)
        best_h = H0
        segment_end = None
        progress_made = False

        while open_list:
            _, g, x, y = heappop(open_list)
            if g_cost[(x, y)] < g:
                continue
          
            h_cur = manhattan(x, y, gx, gy)
            if h_cur < best_h:
                best_h = h_cur
                best_node = (x, y)
                progress_made = True

            # Enhanced early stopping conditions
            if (H0 - h_cur > sw) or (g > block_size//2):
                segment_end = (x, y)
                break

            # Explore neighbors with global bounds check
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                nx, ny = x+dx, y+dy
                if 0 <= nx < cols and 0 <= ny < rows and grid[ny][nx] == 0:
                    ng = g + 1
                    if (nx, ny) not in g_cost or ng < g_cost[(nx, ny)]:
                        g_cost[(nx, ny)] = ng
                        came_from[(nx, ny)] = (x, y)
                        heappush(open_list, (ng + manhattan(nx, ny, gx, gy), ng, nx, ny))

        # Fallback to best node if no early stop
        if segment_end is None:
            if progress_made:
                segment_end = best_node
            else:
                # Find closest accessible point to goal
                if came_from:
                    best_node = min(came_from.keys(), 
                                   key=lambda pos: manhattan(pos[0], pos[1], gx, gy))
                    segment_end = best_node
                else:
                    break

        seg_path = []
        node = segment_end
        while node is not None:
            seg_path.append(node)
            node = came_from.get(node)
        seg_path.reverse()
        path.extend(seg_path if first_segment else seg_path[1:])
        first_segment = False

        if segment_end == (gx, gy):
            return path

        current_start = segment_end

    return path
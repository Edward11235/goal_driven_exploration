from collections import deque
from heapq import heappush, heappop

def plan_path(grid, start, goal, region_size=100):
    rows, cols = len(grid), len(grid[0])
    rx_count = (cols + region_size - 1) // region_size
    ry_count = (rows + region_size - 1) // region_size

    # Create bidirectional region ID mapping
    id_region = {}
    for rj in range(ry_count):
        for ri in range(rx_count):
            rid = rj * rx_count + ri
            id_region[rid] = (ri, rj)

    # Compute region indices for start and goal
    sx, sy = start; gx, gy = goal
    start_region = (sx // region_size, sy // region_size)
    goal_region = (gx // region_size, gy // region_size)

    # Precompute representative points for each region
    region_points = {}
    for rj in range(ry_count):
        for ri in range(rx_count):
            x0 = ri * region_size
            y0 = rj * region_size
            x1 = min(cols-1, x0 + region_size - 1)
            y1 = min(rows-1, y0 + region_size - 1)
            w = x1 - x0 + 1
            h = y1 - y0 + 1
            pts = [
                (x0, y0), (x1, y0), (x0, y1), (x1, y1),
                (x0 + w//2, y0), (x0 + w//2, y1),
                (x0, y0 + h//2), (x1, y0 + h//2),
                (x0 + w//2, y0 + h//2)
            ]
            region_points[(ri, rj)] = pts

    # Helper to get region ID
    def region_id(region_coord):
        ri, rj = region_coord
        return rj * rx_count + ri

    # Build abstract graph
    adj = {}
    def add_edge(u, v, cost):
        adj.setdefault(u, []).append((v, cost))
        adj.setdefault(v, []).append((u, cost))

    # 1. Intra-region connectivity
    for rj in range(ry_count):
        for ri in range(rx_count):
            region = (ri, rj)
            pts = region_points[region]
            region_key = region_id(region)
          
            for i, (sx_i, sy_i) in enumerate(pts):
                if grid[sy_i][sx_i] != 0:
                    continue
              
                # BFS for intra-region paths
                x0 = ri * region_size
                y0 = rj * region_size
                x1 = min(cols-1, x0 + region_size - 1)
                y1 = min(rows-1, y0 + region_size - 1)
              
                dist = {(sx_i, sy_i): 0}
                dq = deque([(sx_i, sy_i)])
                found = {}
              
                while dq:
                    x, y = dq.popleft()
                    current_dist = dist[(x, y)]
                  
                    # Check if current position is a representative point
                    for j, (px, py) in enumerate(pts):
                        if j != i and (x, y) == (px, py):
                            found[j] = current_dist
                  
                    # Explore neighbors
                    for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                        nx, ny = x + dx, y + dy
                        if (nx < x0 or nx > x1 or ny < y0 or ny > y1 or
                            (nx, ny) in dist or grid[ny][nx] != 0):
                            continue
                        dist[(nx, ny)] = current_dist + 1
                        dq.append((nx, ny))
              
                # Add edges for found connections
                for j, d in found.items():
                    u_node = (region_key, i)
                    v_node = (region_key, j)
                    add_edge(u_node, v_node, d)

    # 2. Inter-region connectivity
    # Horizontal connections
    for rj in range(ry_count):
        for ri in range(rx_count - 1):
            regA = (ri, rj)
            regB = (ri+1, rj)
            ptsA = region_points[regA]
            ptsB = region_points[regB]
          
            # Right edge of A to left edge of B
            iA = 7  # Right-middle
            iB = 6  # Left-middle
            ax, ay = ptsA[iA]
            bx, by = ptsB[iB]
          
            if (ax + 1 == bx and ay == by and 
                grid[ay][ax] == 0 and grid[by][bx] == 0):
                u = (region_id(regA), iA)
                v = (region_id(regB), iB)
                add_edge(u, v, 1)

    # Vertical connections
    for rj in range(ry_count - 1):
        for ri in range(rx_count):
            regA = (ri, rj)
            regB = (ri, rj+1)
            ptsA = region_points[regA]
            ptsB = region_points[regB]
          
            # Bottom edge of A to top edge of B
            iA = 5  # Bottom-middle
            iB = 4  # Top-middle
            ax, ay = ptsA[iA]
            bx, by = ptsB[iB]
          
            if (ax == bx and ay + 1 == by and 
                grid[ay][ax] == 0 and grid[by][bx] == 0):
                u = (region_id(regA), iA)
                v = (region_id(regB), iB)
                add_edge(u, v, 1)

    # 3. Connect start/goal to abstract graph
    start_node = 'START'
    goal_node = 'GOAL'
  
    # Connect start to its region
    sr_pts = region_points[start_region]
    for j, (px, py) in enumerate(sr_pts):
        if grid[py][px] != 0:
            continue
      
        # BFS from start to representative point
        dq = deque([start])
        dist = {start: 0}
        found = False
      
        while dq and not found:
            x, y = dq.popleft()
            if (x, y) == (px, py):
                found = True
                d = dist[(x, y)]
                add_edge(start_node, (region_id(start_region), j), d)
                add_edge((region_id(start_region), j), start_node, d)
            for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                nx, ny = x + dx, y + dy
                if (nx < start_region[0]*region_size or 
                    nx >= (start_region[0]+1)*region_size or
                    ny < start_region[1]*region_size or 
                    ny >= (start_region[1]+1)*region_size):
                    continue
                if (nx, ny) not in dist and grid[ny][nx] == 0:
                    dist[(nx, ny)] = dist[(x, y)] + 1
                    dq.append((nx, ny))

    # Connect goal to its region
    gr_pts = region_points[goal_region]
    for j, (px, py) in enumerate(gr_pts):
        if grid[py][px] != 0:
            continue
      
        # BFS from representative point to goal
        dq = deque([(px, py)])
        dist = {(px, py): 0}
        found = False
      
        while dq and not found:
            x, y = dq.popleft()
            if (x, y) == goal:
                found = True
                d = dist[(x, y)]
                add_edge((region_id(goal_region), j), goal_node, d)
                add_edge(goal_node, (region_id(goal_region), j), d)
            for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                nx, ny = x + dx, y + dy
                if (nx < goal_region[0]*region_size or 
                    nx >= (goal_region[0]+1)*region_size or
                    ny < goal_region[1]*region_size or 
                    ny >= (goal_region[1]+1)*region_size):
                    continue
                if (nx, ny) not in dist and grid[ny][nx] == 0:
                    dist[(nx, ny)] = dist[(x, y)] + 1
                    dq.append((nx, ny))

    # 4. Hierarchical A* search
    open_heap = []
    heappush(open_heap, (0, start_node))
    came_from = {start_node: None}
    cost_so_far = {start_node: 0}
  
    while open_heap:
        current_f, current = heappop(open_heap)
      
        if current == goal_node:
            break
      
        if current_f > cost_so_far.get(current, float('inf')):
            continue
      
        for neighbor, cost in adj.get(current, []):
            new_cost = cost_so_far[current] + cost
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost
              
                # Heuristic calculation
                if neighbor not in (start_node, goal_node):
                    reg_id = neighbor[0]
                    reg_coord = id_region[reg_id]
                    h = (abs(reg_coord[0] - goal_region[0]) + 
                         abs(reg_coord[1] - goal_region[1])) * region_size
                    priority += h
              
                heappush(open_heap, (priority, neighbor))
                came_from[neighbor] = current

    # Path reconstruction
    if goal_node not in came_from:
        return []
  
    # Build abstract path
    path = []
    current = goal_node
    while current:
        path.append(current)
        current = came_from.get(current)
    path.reverse()

    # 5. Path refinement
    refined_path = [start]
    for i in range(1, len(path)):
        prev = path[i-1]
        curr = path[i]
      
        if prev == start_node:
            # Handle start cluster
            target = region_points[start_region][curr[1]]
            # BFS from start to target
            dq = deque([start])
            prev_map = {start: None}
            while dq:
                x, y = dq.popleft()
                if (x, y) == target:
                    break
                for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                    nx, ny = x + dx, y + dy
                    if (nx < start_region[0]*region_size or 
                        nx >= (start_region[0]+1)*region_size or
                        ny < start_region[1]*region_size or 
                        ny >= (start_region[1]+1)*region_size):
                        continue
                    if (nx, ny) not in prev_map and grid[ny][nx] == 0:
                        prev_map[(nx, ny)] = (x, y)
                        dq.append((nx, ny))
            # Reconstruct path segment
            segment = []
            node = target
            while node:
                segment.append(node)
                node = prev_map.get(node)
            refined_path += segment[::-1][1:]
          
        elif curr == goal_node:
            # Handle goal cluster
            source = region_points[goal_region][prev[1]]
            # BFS to goal
            dq = deque([source])
            prev_map = {source: None}
            while dq:
                x, y = dq.popleft()
                if (x, y) == goal:
                    break
                for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                    nx, ny = x + dx, y + dy
                    if (nx < goal_region[0]*region_size or 
                        nx >= (goal_region[0]+1)*region_size or
                        ny < goal_region[1]*region_size or 
                        ny >= (goal_region[1]+1)*region_size):
                        continue
                    if (nx, ny) not in prev_map and grid[ny][nx] == 0:
                        prev_map[(nx, ny)] = (x, y)
                        dq.append((nx, ny))
            # Reconstruct path segment
            segment = []
            node = goal
            while node:
                segment.append(node)
                node = prev_map.get(node)
            refined_path += segment[::-1][1:]
          
        else:
            # Handle regular cluster transitions
            current_pos = refined_path[-1]
            target = region_points[id_region[curr[0]]][curr[1]]
            # Straight-line path between regions
            dx = target[0] - current_pos[0]
            dy = target[1] - current_pos[1]
            x_step = 1 if dx > 0 else -1 if dx < 0 else 0
            y_step = 1 if dy > 0 else -1 if dy < 0 else 0
          
            x, y = current_pos
            while x != target[0]:
                x += x_step
                refined_path.append((x, y))
            while y != target[1]:
                y += y_step
                refined_path.append((x, y))

    return refined_path
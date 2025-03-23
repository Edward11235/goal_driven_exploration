# path_planner_hierarchical.py
from collections import deque
from heapq import heappush, heappop

def plan_path(grid, start, goal, region_size=100):
    rows, cols = len(grid), len(grid[0])
    rx_count = (cols + region_size - 1) // region_size
    ry_count = (rows + region_size - 1) // region_size

    # Compute region indices for start and goal
    sx, sy = start; gx, gy = goal
    start_region = (sx // region_size, sy // region_size)
    goal_region  = (gx // region_size, gy // region_size)

    # Precompute representative points for each region (8 around edges + center)
    region_points = {}
    for rj in range(ry_count):
        for ri in range(rx_count):
            x0 = ri * region_size
            y0 = rj * region_size
            x1 = min(cols-1, x0 + region_size - 1)
            y1 = min(rows-1, y0 + region_size - 1)
            w = x1 - x0 + 1; h = y1 - y0 + 1
            # 9 points: corners, edge midpoints, center
            pts = [
                (x0, y0), (x1, y0), (x0, y1), (x1, y1),
                (x0 + w//2, y0), (x0 + w//2, y1),
                (x0, y0 + h//2), (x1, y0 + h//2),
                (x0 + w//2, y0 + h//2)
            ]
            region_points[(ri, rj)] = pts

    # Helper to get region id (for use as node in abstract graph)
    def region_id(region_coord):
        ri, rj = region_coord
        return rj * rx_count + ri

    # Build abstract graph adjacency list
    adj = {}
    def add_edge(node_u, node_v, cost):
        adj.setdefault(node_u, []).append((node_v, cost))

    # 1. Intra-region connectivity: BFS within each cluster between rep points
    for rj in range(ry_count):
        for ri in range(rx_count):
            region = (ri, rj)
            pts = region_points[region]
            region_key = region_id(region)
            # BFS from each rep point i to find distance to others in same region
            for i, (sx_i, sy_i) in enumerate(pts):
                if grid[sy_i][sx_i] != 0: 
                    continue  # skip if rep point is on an obstacle
                # BFS inside region bounds
                x0 = ri * region_size; y0 = rj * region_size
                x1 = min(cols-1, x0 + region_size - 1)
                y1 = min(rows-1, y0 + region_size - 1)
                dist = { (sx_i, sy_i): 0 }
                dq = deque([(sx_i, sy_i)])
                # Track distances to other rep points
                found = {}
                while dq:
                    x, y = dq.popleft()
                    d = dist[(x, y)]
                    # If this location is another rep point
                    for j, (px, py) in enumerate(pts):
                        if j != i and (x, y) == (px, py):
                            found[j] = d
                    if len(found) == len(pts) - 1:
                        # All reachable rep points found
                        pass  # (we could break early, but we let BFS run out for completeness)
                    for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                        nx, ny = x+dx, y+dy
                        if nx < x0 or nx > x1 or ny < y0 or ny > y1:
                            continue
                        if (nx, ny) not in dist and grid[ny][nx] == 0:
                            dist[(nx, ny)] = d + 1
                            dq.append((nx, ny))
                # Add edges for found connections
                for j, d in found.items():
                    u = (region_key, i)   # node = (region_id, rep_index)
                    v = (region_key, j)
                    add_edge(u, v, d)
                    add_edge(v, u, d)

    # 2. Inter-region connectivity: connect adjacent clusters via matching rep points
    # Horizontal neighbors (right-mid of left region to left-mid of right region)
    for rj in range(ry_count):
        for ri in range(rx_count - 1):
            regA = (ri, rj); regB = (ri+1, rj)
            ptsA = region_points[regA]; ptsB = region_points[regB]
            # Right-mid index = 7, Left-mid index = 6
            iA, iB = 7, 6
            Ax, Ay = ptsA[iA]; Bx, By = ptsB[iB]
            if Ax + 1 == Bx and Ay == By and grid[Ay][Ax] == 0 and grid[By][Bx] == 0:
                u = (region_id(regA), iA); v = (region_id(regB), iB)
                add_edge(u, v, 1); add_edge(v, u, 1)
    # Vertical neighbors (bottom-mid of top region to top-mid of bottom region)
    for rj in range(ry_count - 1):
        for ri in range(rx_count):
            regA = (ri, rj); regB = (ri, rj+1)
            ptsA = region_points[regA]; ptsB = region_points[regB]
            iA, iB = 5, 4
            Ax, Ay = ptsA[iA]; Bx, By = ptsB[iB]
            if Ax == Bx and Ay + 1 == By and grid[Ay][Ax] == 0 and grid[By][Bx] == 0:
                u = (region_id(regA), iA); v = (region_id(regB), iB)
                add_edge(u, v, 1); add_edge(v, u, 1)

    # 3. Connect start and goal to their regions' rep nodes
    start_node = 'START'; goal_node = 'GOAL'
    # BFS from start to each rep in start_region
    sr_pts = region_points[start_region]
    sx0 = start_region[0] * region_size; sy0 = start_region[1] * region_size
    sx1 = min(cols-1, sx0 + region_size - 1); sy1 = min(rows-1, sy0 + region_size - 1)
    for j, (px, py) in enumerate(sr_pts):
        if grid[py][px] != 0:
            continue
        dq = deque([start]); dist = { start: 0 }
        while dq:
            x, y = dq.popleft()
            if (x, y) == (px, py):
                d = dist[(x, y)]
                u = start_node; v = (region_id(start_region), j)
                add_edge(u, v, d); add_edge(v, u, d)
                break
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                nx, ny = x+dx, y+dy
                if nx < sx0 or nx > sx1 or ny < sy0 or ny > sy1:
                    continue
                if (nx, ny) not in dist and grid[ny][nx] == 0:
                    dist[(nx, ny)] = dist[(x, y)] + 1
                    dq.append((nx, ny))
    # BFS from each rep in goal_region to goal
    gr_pts = region_points[goal_region]
    gx0 = goal_region[0] * region_size; gy0 = goal_region[1] * region_size
    gx1 = min(cols-1, gx0 + region_size - 1); gy1 = min(rows-1, gy0 + region_size - 1)
    for i, (px, py) in enumerate(gr_pts):
        if grid[py][px] != 0:
            continue
        dq = deque([(px, py)]); dist = { (px, py): 0 }
        while dq:
            x, y = dq.popleft()
            if (x, y) == (gx, gy):
                d = dist[(x, y)]
                u = (region_id(goal_region), i); v = goal_node
                add_edge(u, v, d); add_edge(v, u, d)
                break
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                nx, ny = x+dx, y+dy
                if nx < gx0 or nx > gx1 or ny < gy0 or ny > gy1:
                    continue
                if (nx, ny) not in dist and grid[ny][nx] == 0:
                    dist[(nx, ny)] = dist[(x, y)] + 1
                    dq.append((nx, ny))

    # 4. A* search on the abstract graph (nodes are cluster rep points)
    open_list = [(0, start_node)]
    came_from = { start_node: None }
    cost_so_far = { start_node: 0 }
    while open_list:
        f, u = heappop(open_list)
        if u == goal_node:
            break
        if cost_so_far[u] < f - 0:  # adjust if f included heuristic
            continue
        for v, w in adj.get(u, []):
            new_cost = cost_so_far[u] + w
            if v not in cost_so_far or new_cost < cost_so_far[v]:
                cost_so_far[v] = new_cost
                came_from[v] = u
                # Heuristic: use Manhattan distance between cluster indices as heuristic
                if v in (start_node, goal_node) or u in (start_node, goal_node):
                    h = 0
                else:
                    # both u and v are tuples (region_id, rep_idx)
                    reg_v = id_region[v[0]]; reg_goal = goal_region
                    h = abs(reg_v[0] - reg_goal[0]) + abs(reg_v[1] - reg_goal[1])
                heappush(open_list, (new_cost + h, v))

    # Reconstruct abstract path (sequence of rep nodes from start to goal)
    if goal_node not in came_from:
        return []  # no path found
    abstract_path = []
    node = goal_node
    while node is not None:
        abstract_path.append(node)
        node = came_from[node]
    abstract_path.reverse()

    # 5. Refine abstract path into actual cell-by-cell path
    full_path = []
    current_pos = start
    for k in range(1, len(abstract_path)):
        u = abstract_path[k-1]; v = abstract_path[k]
        if u == start_node:
            # Transition from start to a cluster rep point v
            target = region_points[start_region][v[1]]
            # BFS within start region to get path from current_pos to target
            dq = deque([current_pos]); prev = { current_pos: None }
            while dq:
                x, y = dq.popleft()
                if (x, y) == target:
                    break
                for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                    nx, ny = x+dx, y+dy
                    if nx < sx0 or nx > sx1 or ny < sy0 or ny > sy1:
                        continue
                    if (nx, ny) not in prev and grid[ny][nx] == 0:
                        prev[(nx, ny)] = (x, y)
                        dq.append((nx, ny))
            # Reconstruct and append this segment
            node = target
            segment = []
            while node is not None:
                segment.append(node)
                node = prev[node]
            segment.reverse()
            full_path.extend(segment)
            current_pos = target
        elif v == goal_node:
            # Transition from a rep point u in goal region to the actual goal
            region = goal_region
            source = region_points[region][u[1]]
            dq = deque([source]); prev = { source: None }
            while dq:
                x, y = dq.popleft()
                if (x, y) == (gx, gy):
                    break
                for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                    nx, ny = x+dx, y+dy
                    if nx < gx0 or nx > gx1 or ny < gy0 or ny > gy1:
                        continue
                    if (nx, ny) not in prev and grid[ny][nx] == 0:
                        prev[(nx, ny)] = (x, y)
                        dq.append((nx, ny))
            # Reconstruct path to goal and append (skipping the first node, which is u)
            node = (gx, gy)
            segment = []
            while node is not None:
                segment.append(node)
                node = prev[node]
            segment.reverse()
            full_path.extend(segment[1:])
            current_pos = (gx, gy)
        else:
            # Transition either within the same region or between adjacent regions
            reg_u_id, rep_u = u; reg_v_id, rep_v = v
            reg_u = (reg_u_id % rx_count, reg_u_id // rx_count)
            reg_v = (reg_v_id % rx_count, reg_v_id // rx_count)
            if reg_u == reg_v:
                # Move inside one cluster from rep_u to rep_v
                x0 = reg_u[0]*region_size; y0 = reg_u[1]*region_size
                x1 = min(cols-1, x0 + region_size - 1); y1 = min(rows-1, y0 + region_size - 1)
                src = region_points[reg_u][rep_u]; dest = region_points[reg_u][rep_v]
                dq = deque([src]); prev = { src: None }
                while dq:
                    x, y = dq.popleft()
                    if (x, y) == dest:
                        break
                    for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                        nx, ny = x+dx, y+dy
                        if nx < x0 or nx > x1 or ny < y0 or ny > y1:
                            continue
                        if (nx, ny) not in prev and grid[ny][nx] == 0:
                            prev[(nx, ny)] = (x, y)
                            dq.append((nx, ny))
                # Append path inside the region (skip the first point to avoid duplicate)
                node = dest
                segment = []
                while node is not None:
                    segment.append(node)
                    node = prev[node]
                segment.reverse()
                full_path.extend(segment[1:])
                current_pos = dest
            else:
                # Adjacent regions: rep_u and rep_v share a border. Just step across.
                pt_u = region_points[reg_u][rep_u]
                pt_v = region_points[reg_v][rep_v]
                # Move from current_pos to the border point in region U (pt_u)
                # Since region U is free, we can move in a straight line (Manhattan path)
                cx, cy = current_pos; bx, by = pt_u
                # Horizontal then vertical (or vice versa) move inside free region
                if cx != bx:
                    step = 1 if bx > cx else -1
                    for x in range(cx + step, bx + step, step):
                        full_path.append((x, cy))
                if cy != by:
                    step = 1 if by > cy else -1
                    for y in range(cy + step, by + step, step):
                        full_path.append((bx, y))
                # Now step across the border into region V
                full_path.append(pt_v)
                current_pos = pt_v
    return full_path

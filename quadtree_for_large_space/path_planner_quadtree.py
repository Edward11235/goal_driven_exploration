# path_planner_quadtree.py
from heapq import heappush, heappop

class QuadNode:
    def __init__(self, x, y, w, h, free=False):
        self.x = x       # top-left corner
        self.y = y
        self.w = w       # width  (in cells)
        self.h = h       # height (in cells)
        self.free = free # True if completely free, False if completely blocked (leaf)
        self.children = []  # list of 4 children if subdivided

def build_quadtree(grid, x, y, w, h):
    # Base case: area of size 1 or homogeneous area
    if w <= 0 or h <= 0:
        return None
    all_free = True
    all_blocked = True
    for j in range(y, y+h):
        for i in range(x, x+w):
            if grid[j][i] == 1:  # obstacle
                all_free = False
            else:                # free cell
                all_blocked = False
            if not all_free and not all_blocked:
                break
        if not all_free and not all_blocked:
            break

    node = QuadNode(x, y, w, h)
    if all_free:
        node.free = True
        return node
    if all_blocked:
        node.free = False
        return node

    # Subdivide into 4 quadrants
    w2 = w // 2; h2 = h // 2
    # Top-left
    node.children.append(build_quadtree(grid, x, y, w2, h2))
    # Top-right
    node.children.append(build_quadtree(grid, x+w2, y, w - w2, h2))
    # Bottom-left
    node.children.append(build_quadtree(grid, x, y+h2, w2, h - h2))
    # Bottom-right
    node.children.append(build_quadtree(grid, x+w2, y+h2, w - w2, h - h2))
    # Remove None children (in case of odd splits where one dimension is 0)
    node.children = [child for child in node.children if child is not None]
    # If all children are leaves and have the same free status, merge them
    if node.children and all(child.children == [] for child in node.children):
        if all(child.free for child in node.children):
            node.children = []
            node.free = True
            return node
        if all(not child.free for child in node.children):
            node.children = []
            node.free = False
            return node
    return node

def get_free_leaves(node):
    # Traverse quadtree to get all free leaf nodes
    leaves = []
    if not node:
        return leaves
    if node.children == []:
        if node.free:
            leaves.append(node)
        return leaves
    for child in node.children:
        leaves.extend(get_free_leaves(child))
    return leaves

def plan_path(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    # Build quadtree for the entire grid
    root = build_quadtree(grid, 0, 0, cols, rows)
    free_leaves = get_free_leaves(root)
    if not free_leaves:
        return []  # no free space at all

    # Map each free leaf to an id and label grid cells for adjacency detection
    leaf_id = {}
    id_to_leaf = {}
    label = [[-1] * cols for _ in range(rows)]
    for idx, leaf in enumerate(free_leaves):
        leaf_id[leaf] = idx
        id_to_leaf[idx] = leaf
        # Mark all cells covered by this leaf with its id
        for j in range(leaf.y, leaf.y + leaf.h):
            for i in range(leaf.x, leaf.x + leaf.w):
                label[j][i] = idx

    start_id = label[start[1]][start[0]]
    goal_id  = label[goal[1]][goal[0]]
    if start_id == -1 or goal_id == -1:
        return []  # either start or goal is in an obstacle

    # Build adjacency list of free leaf graph
    neighbors = { idx: set() for idx in id_to_leaf }
    # Check horizontal and vertical adjacencies in the original grid
    for y in range(rows):
        for x in range(cols-1):
            a, b = label[y][x], label[y][x+1]
            if a != b and a != -1 and b != -1:
                neighbors[a].add(b)
                neighbors[b].add(a)
    for y in range(rows-1):
        for x in range(cols):
            a, b = label[y][x], label[y+1][x]
            if a != b and a != -1 and b != -1:
                neighbors[a].add(b)
                neighbors[b].add(a)

    # A* search on the leaf adjacency graph
    open_list = [(0, start_id)]
    came_from = { start_id: None }
    g_cost = { start_id: 0 }
    # Heuristic: Manhattan distance between leaf centers (in block units)
    def leaf_center(leaf):
        cx = leaf.x + leaf.w//2
        cy = leaf.y + leaf.h//2
        return (cx, cy)
    goal_center = leaf_center(id_to_leaf[goal_id])

    while open_list:
        f, lid = heappop(open_list)
        if lid == goal_id:
            break
        if g_cost.get(lid, float('inf')) < f - 0:  # ignore outdated entry
            continue
        for nbr in neighbors[lid]:
            ng = g_cost[lid] + 1  # each neighbor transition = 1 step (abstract)
            if ng < g_cost.get(nbr, float('inf')):
                g_cost[nbr] = ng
                came_from[nbr] = lid
                # Heuristic = Manhattan distance between block centers (in grid coords)
                nbr_center = leaf_center(id_to_leaf[nbr])
                h = abs(nbr_center[0] - goal_center[0]) + abs(nbr_center[1] - goal_center[1])
                heappush(open_list, (ng + h, nbr))

    if goal_id not in came_from:
        return []  # no path found

    # Reconstruct leaf-to-leaf path (sequence of leaf IDs)
    leaf_path = []
    lid = goal_id
    while lid is not None:
        leaf_path.append(lid)
        lid = came_from.get(lid)
    leaf_path.reverse()

    # Refine leaf path to actual grid path
    path = []
    current_pos = start
    for idx in range(len(leaf_path)-1):
        leafA = id_to_leaf[leaf_path[idx]]
        leafB = id_to_leaf[leaf_path[idx+1]]
        # Determine the shared border between leafA and leafB
        # Compute overlapping edge segment
        # Leaf coordinates: top-left (Ax,Ay) to bottom-right (Ax+w-1, Ay+h-1)
        Ax0, Ay0 = leafA.x, leafA.y
        Ax1, Ay1 = leafA.x + leafA.w - 1, leafA.y + leafA.h - 1
        Bx0, By0 = leafB.x, leafB.y
        Bx1, By1 = leafB.x + leafB.w - 1, leafB.y + leafB.h - 1
        # Determine which side they touch
        if Ax1 + 1 == Bx0 or Bx1 + 1 == Ax0:  # horizontal neighbors
            # Shared vertical segment (overlap in y)
            overlap_y0 = max(Ay0, By0)
            overlap_y1 = min(Ay1, By1)
            cross_y = (overlap_y0 + overlap_y1) // 2
            # If one block is directly to the right of the other
            cross_xA = Ax1 if Ax1 + 1 == Bx0 else Ax0  # border x in A
            cross_xB = Bx0 if Ax1 + 1 == Bx0 else Bx1  # border x in B
        else:  # vertical neighbors
            overlap_x0 = max(Ax0, Bx0)
            overlap_x1 = min(Ax1, Bx1)
            cross_x = (overlap_x0 + overlap_x1) // 2
            cross_yA = Ay1 if Ay1 + 1 == By0 else Ay0
            cross_yB = By0 if Ay1 + 1 == By0 else By1
            cross_y = None  # will use cross_yA and cross_yB
        # Move inside leafA from current_pos to the crossing point on the border
        cx, cy = current_pos
        if 'cross_x' in locals():
            # Horizontal movement to align x
            if cx < cross_x:
                for x in range(cx+1, cross_x+1):
                    path.append((x, cy))
            elif cx > cross_x:
                for x in range(cx-1, cross_x-1, -1):
                    path.append((x, cy))
            if 'cross_y' in locals() and cross_y is not None:
                # Then vertical movement to align y
                if cy < cross_y:
                    for y in range(cy+1, cross_y+1):
                        path.append((cross_x, y))
                elif cy > cross_y:
                    for y in range(cy-1, cross_y-1, -1):
                        path.append((cross_x, y))
        else:
            # Horizontal alignment
            if cx < cross_x:
                for x in range(cx+1, cross_x+1):
                    path.append((x, cy))
            elif cx > cross_x:
                for x in range(cx-1, cross_x-1, -1):
                    path.append((x, cy))
            # Vertical alignment to border
            if cy < cross_yA:
                for y in range(cy+1, cross_yA+1):
                    path.append((cross_x, y))
            elif cy > cross_yA:
                for y in range(cy-1, cross_yA-1, -1):
                    path.append((cross_x, y))
        # Now step across from leafA to leafB
        if 'cross_y' in locals() and cross_y is not None:
            # Horizontal neighbor case
            path.append((cross_xB, cross_y))
            current_pos = (cross_xB, cross_y)
        else:
            # Vertical neighbor case
            path.append((cross_x, cross_yB))
            current_pos = (cross_x, cross_yB)
        # Clean up temp variables for next iteration
        if 'cross_x' in locals(): del cross_x
        if 'cross_y' in locals(): del cross_y

    # Finally, handle movement within the last leaf to the goal
    cx, cy = current_pos; gx, gy = goal
    # Move horizontally then vertically (within free block, this is valid)
    if cx < gx:
        for x in range(cx+1, gx+1):
            path.append((x, cy))
    elif cx > gx:
        for x in range(cx-1, gx-1, -1):
            path.append((x, cy))
    if cy < gy:
        for y in range(cy+1, gy+1):
            path.append((gx, y))
    elif cy > gy:
        for y in range(cy-1, gy-1, -1):
            path.append((gx, y))
    return path

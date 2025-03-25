
import heapq

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(start, goal, obstacles, grid_size):
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start, None))
    came_from = {}
    cost_so_far = {start: 0}

    while open_set:
        _, cost, current, parent = heapq.heappop(open_set)
        came_from[current] = parent

        if current == goal:
            path = []
            while current:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            next_node = (current[0] + dx, current[1] + dy)
            if 0 <= next_node[0] < grid_size and 0 <= next_node[1] < grid_size and next_node not in obstacles:
                new_cost = cost + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(next_node, goal)
                    heapq.heappush(open_set, (priority, new_cost, next_node, current))

    return None

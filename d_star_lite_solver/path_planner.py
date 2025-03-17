# path_planner.py (updated with D* Lite)
"""
Path planning module: contains D* Lite algorithm for dynamic pathfinding.
"""

import heapq

class DStarLite:
    def __init__(self, start, goal, grid):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.km = 0
        self.U = []  # Priority queue stores tuples of (key, node)
        self.rhs = {self.goal: 0}
        self.g = {self.goal: float('inf')}
        self.key_mod = 0
        self.add_or_update(self.goal, self.calculate_key(self.goal))
  
    def calculate_key(self, node):
        g_rhs = min(self.g.get(node, float('inf')), self.rhs.get(node, float('inf')))
        h = self.heuristic(self.start, node)
        return (g_rhs + h + self.km, g_rhs)  # Return tuple as key
  
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
  
    def update_vertex(self, u):
        if u != self.goal:
            self.rhs[u] = min([self.g.get((u[0]+dx, u[1]+dy), float('inf')) + 1 
                             for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]
                             if self.is_traversable((u[0]+dx, u[1]+dy))], 
                             default=float('inf'))
      
        if u in self.g:
            self.remove(u)
          
        if self.g.get(u, float('inf')) != self.rhs.get(u, float('inf')):
            self.add_or_update(u, self.calculate_key(u))
  
    def is_traversable(self, cell):
        x, y = cell
        if not (0 <= x < len(self.grid) and 0 <= y < len(self.grid[0])):
            return False
        return self.grid[x][y] != 1
  
    def compute_shortest_path(self):
        while self.U:
            current_key = self.U[0][0]  # Get full key tuple
            start_key = self.calculate_key(self.start)
          
            # Compare keys lexicographically
            if current_key > start_key and \
               self.rhs.get(self.start, float('inf')) == self.g.get(self.start, float('inf')):
                break
              
            k_old = self.U[0][0]
            u = heapq.heappop(self.U)[1]
            k_new = self.calculate_key(u)
          
            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
            elif self.g.get(u, float('inf')) > self.rhs.get(u, float('inf')):
                self.g[u] = self.rhs[u]
                for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                    self.update_vertex((u[0]+dx, u[1]+dy))
            else:
                self.g[u] = float('inf')
                for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                    self.update_vertex((u[0]+dx, u[1]+dy))
                self.update_vertex(u)
  
    def plan_path(self):
        path = []
        current = self.start
        if self.g.get(current, float('inf')) == float('inf'):
            return None
          
        while current != self.goal:
            path.append(current)
            neighbors = [(current[0]+dx, current[1]+dy) for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]]
            valid_neighbors = [n for n in neighbors if self.is_traversable(n)]
            if not valid_neighbors:
                return None
            current = min(valid_neighbors, key=lambda n: self.g.get(n, float('inf')))
      
        path.append(self.goal)
        return path
  
    def add_or_update(self, node, key):
        entries = [entry for entry in self.U if entry[1] == node]
        if entries:
            self.U.remove(entries[0])
        heapq.heappush(self.U, (key[0], node))
  
    def remove(self, node):
        entries = [entry for entry in self.U if entry[1] == node]
        if entries:
            self.U.remove(entries[0])
            heapq.heapify(self.U)

    def handle_changed_edges(self, cells):
        for cell in cells:
            self.update_vertex(cell)
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                neighbor = (cell[0]+dx, cell[1]+dy)
                if 0 <= neighbor[0] < len(self.grid) and 0 <= neighbor[1] < len(self.grid[0]):
                    self.update_vertex(neighbor)
        self.compute_shortest_path()

    def update_start(self, new_start):
        self.km += self.heuristic(self.start, new_start)
        self.start = new_start
        self.compute_shortest_path()

    def add_or_update(self, node, key):
        """Store full key tuple in the priority queue"""
        entries = [entry for entry in self.U if entry[1] == node]
        if entries:
            self.U.remove(entries[0])
        heapq.heappush(self.U, (key, node))  # Store full key tuple

    def remove(self, node):
        entries = [entry for entry in self.U if entry[1] == node]
        if entries:
            self.U.remove(entries[0])
            heapq.heapify(self.U)
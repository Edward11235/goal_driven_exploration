import numpy as np

def generate_maze(size=1000):
    grid = np.zeros((size, size), dtype=int)
  
    moves = ['D'] * (size - 1) + ['R'] * (size - 1)
    np.random.shuffle(moves)
    path_coords = [(0, 0)]
    r, c = 0, 0
    for move in moves:
        if move == 'D':
            r += 1
        else:
            c += 1
        path_coords.append((r, c))
    for (pr, pc) in path_coords:
        grid[pr, pc] = 0
  
    obstacle_mask = (np.random.rand(size, size) < 0.3)
    grid[obstacle_mask] = 1
    for (pr, pc) in path_coords:
        grid[pr, pc] = 0
  
    return grid

if __name__ == "__main__":
    maze = generate_maze()
    print("Maze generated with shape:", maze.shape)
    print("Obstacle percentage:", maze.sum() / maze.size * 100, "%")

import numpy as np
import matplotlib.pyplot as plt
from maze_generator import Maze
from robot import Robot
from matplotlib.colors import ListedColormap

use_custom_map = True
map_file_name = "./custom_map.npz"
GRID_SIZE = 1000
algorithm_type = "astar"

def load_custom_map(map_file):
    data = np.load(map_file, allow_pickle=True)
    grid = data['grid']
    start = tuple(data['start'])
    goal = tuple(data['goal'])
    moving_obstacles = [{
        'path': obs.tolist(),
        'current_index': 0
    } for obs in data['moving_obstacles']]
    return grid, start, goal, moving_obstacles

def run_simulation():
    grid, start, goal, moving_obstacles = load_custom_map(map_file_name)
    maze = Maze(GRID_SIZE, GRID_SIZE)
    maze.grid = np.array(grid.tolist())

    robot = Robot(maze, start, goal, vision_radius=20, algorithm=algorithm_type)

    plt.ion()
    fig, (ax_global, ax_local) = plt.subplots(1, 2, figsize=(14, 7))

    cmap = ListedColormap([
        (0.7, 0.7, 0.7),  # Unknown
        (1.0, 1.0, 1.0),  # Free space
        (0.0, 0.0, 0.0),  # Static obstacle
        (1.0, 0.0, 0.0),  # Robot
        (0.0, 1.0, 0.0),  # Goal
        (1.0, 0.5, 0.0)   # Moving obstacle
    ])

    prev_robot_pos = robot.pos

    for step in range(1000):
        # Move obstacles
        for obs in moving_obstacles:
            prev_idx = obs['current_index']
            obs['current_index'] = (obs['current_index'] + 1) % len(obs['path'])
            px, py = obs['path'][prev_idx]
            nx, ny = obs['path'][obs['current_index']]
            maze.grid[py, px] = 0
            maze.grid[ny, nx] = 1

        robot.sense(prev_robot_pos)

        path = robot.plan_path()
        if not path or len(path) < 2:
            print("Path planning failed or completed!")
            break

        # Validate robot move clearly
        next_pos = path[1]
        if maze.grid[next_pos[1], next_pos[0]] == 0:
            prev_robot_pos = robot.pos
            robot.move_one_step(path)
        else:
            print("Robot attempted to move through wall or obstacle!")
            break

        rx, ry = robot.pos
        gx, gy = robot.goal

        # Global view visualization zoomed in around robot
        global_range = 100
        global_min_x = max(0, rx - global_range)
        global_max_x = min(maze.width, rx + global_range)
        global_min_y = max(0, ry - global_range)
        global_max_y = min(maze.height, ry + global_range)

        ax_global.clear()
        ax_global.set_title(f"Global View (Zoomed) - Step {step}")

        # Start by setting unknown areas as gray
        vis_global = np.full((global_max_y - global_min_y, global_max_x - global_min_x), 0.7)

        # Clearly differentiate static obstacles and free space
        for i in range(global_min_y, global_max_y):
            for j in range(global_min_x, global_max_x):
                if maze.grid[i, j] == 0:
                    vis_global[i - global_min_y, j - global_min_x] = 1  # Free space (white)
                elif maze.grid[i, j] == 1:
                    vis_global[i - global_min_y, j - global_min_x] = 2  # Static obstacles (black)

        # Mark robot and goal clearly
        vis_global[ry - global_min_y, rx - global_min_x] = 3  # Robot (red)
        if global_min_x <= gx < global_max_x and global_min_y <= gy < global_max_y:
            vis_global[gy - global_min_y, gx - global_min_x] = 4  # Goal (green)

        # Explicitly mark moving obstacles clearly
        for obs in moving_obstacles:
            ox, oy = obs['path'][obs['current_index']]
            if global_min_x <= ox < global_max_x and global_min_y <= oy < global_max_y:
                vis_global[oy - global_min_y, ox - global_min_x] = 5  # Moving obstacles (orange)

        ax_global.imshow(vis_global, cmap=cmap, origin='upper')


        # Local view visualization (smaller range clearly around robot)
        local_range = 30
        local_min_x = max(0, rx - local_range)
        local_max_x = min(maze.width, rx + local_range)
        local_min_y = max(0, ry - local_range)
        local_max_y = min(maze.height, ry + local_range)

        ax_local.clear()
        ax_local.set_title("Local View")
        vis_local = np.zeros((local_max_y - local_min_y, local_max_x - local_min_x))
        for i in range(local_min_y, local_max_y):
            for j in range(local_min_x, local_max_x):
                cell = robot.known_map[i][j]
                if cell == -1:
                    vis_local[i-local_min_y, j-local_min_x] = 0
                elif cell == 0:
                    vis_local[i-local_min_y, j-local_min_x] = 1
                else:
                    vis_local[i-local_min_y, j-local_min_x] = 2
        vis_local[ry - local_min_y, rx - local_min_x] = 3
        if local_min_x <= gx < local_max_x and local_min_y <= gy < local_max_y:
            vis_local[gy - local_min_y, gx - local_min_x] = 4
        for obs in moving_obstacles:
            ox, oy = obs['path'][obs['current_index']]
            if local_min_x <= ox < local_max_x and local_min_y <= oy < local_max_y:
                vis_local[oy - local_min_y, ox - local_min_x] = 5
        ax_local.imshow(vis_local, cmap=cmap, origin='upper')

        plt.pause(0.05)

        if robot.pos == robot.goal:
            print(f"Goal reached in {step} steps.")
            break

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    run_simulation()
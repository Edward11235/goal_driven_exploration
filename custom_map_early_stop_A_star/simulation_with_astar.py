"""
Simulation script: runs the robot in a maze with visualization.
"""
import time
import matplotlib.pyplot as plt
from maze_generator import Maze
from robot import Robot
import numpy as np

use_custom_map = True
map_file_name = "./custom_map.npz"
GRID_SIZE = 1000
algorithm_type = "astar"

def load_custom_map(map_file):
    data = np.load(map_file)
    return data['grid'], data['start'], data['goal']

def run_simulation():
    if use_custom_map:
        # Load grid from file and create Maze instance
        maze_grid, start, goal = load_custom_map(map_file_name)
        maze = Maze(GRID_SIZE, GRID_SIZE)  # Create dummy maze
        maze.grid = maze_grid.tolist()      # Overwrite with actual grid data
        start = tuple(start)
        goal = tuple(goal)
        robot = Robot(maze, start, goal, vision_radius=5, algorithm=algorithm_type)
    else:
        # Original code remains unchanged
        maze = Maze(GRID_SIZE, GRID_SIZE)
        robot = Robot(maze, start=(0,0), goal=(maze.height-1, maze.width-1), 
                    vision_radius=5, algorithm=algorithm_type)

    # Setup interactive plots for local and global views
    plt.ion()  # enable interactive mode
    fig_local, ax_local = plt.subplots(figsize=(5,5))
    fig_global, ax_global = plt.subplots(figsize=(5,5))
    from matplotlib.colors import ListedColormap
    cmap = ListedColormap([
        (0.7, 0.7, 0.7),  # 0: unknown (gray)
        (1.0, 1.0, 1.0),  # 1: known free (white)
        (0.0, 0.0, 0.0),  # 2: wall (black)
        (0.4, 0.8, 1.0),  # 3: visited path (light blue)
        (1.0, 0.0, 0.0),  # 4: robot (red)
        (0.0, 1.0, 0.0)   # 5: goal (green)
    ])
    ax_local.set_title("Local View (100x100 around robot)")
    ax_global.set_title("Global View (Downsampled 1000x1000)")
    ax_local.set_xticks([]); ax_local.set_yticks([])
    ax_global.set_xticks([]); ax_global.set_yticks([])

    # Initialize local view around the start (0,0)
    local_size = 100
    rx, ry = robot.pos
    # Determine bounds of local view (center on robot, adjust for edges)
    local_x_min = max(0, rx - local_size//2)
    local_y_min = max(0, ry - local_size//2)
    local_x_max = min(maze.height, local_x_min + local_size)
    local_y_max = min(maze.width, local_y_min + local_size)
    if local_x_max - local_x_min < local_size:
        local_x_min = max(0, local_x_max - local_size)
    if local_y_max - local_y_min < local_size:
        local_y_min = max(0, local_y_max - local_size)
    init_local_display = [[0] * (local_y_max - local_y_min) for _ in range(local_x_max - local_x_min)]
    img_local = ax_local.imshow(init_local_display, cmap=cmap, vmin=0, vmax=5)

    # Initialize global view by downsampling the maze to 100x100
    downsampled_h = downsampled_w = 100
    block_h = maze.height // downsampled_h  # 10 for 1000x1000
    block_w = maze.width // downsampled_w   # 10 for 1000x1000
    global_base = [[0] * downsampled_w for _ in range(downsampled_h)]
    for i in range(downsampled_h):
        for j in range(downsampled_w):
            # Determine the portion of the maze this pixel represents
            wall_count = 0
            # Count walls in the corresponding block of the actual maze
            for ii in range(i * block_h, min((i+1) * block_h, maze.height)):
                for jj in range(j * block_w, min((j+1) * block_w, maze.width)):
                    if maze.grid[ii][jj] == 1:
                        wall_count += 1
            # Mark as wall if >=30% of block cells are walls
            if use_custom_map == False:
                if wall_count >= int(block_h * block_w * 0.3):
                    global_base[i][j] = 2  # mostly wall in that block
                else:
                    global_base[i][j] = 1  # mostly free in that block
            else:
                if wall_count > 0:
                    global_base[i][j] = 2  # mark as wall if any obstacles present
                else:
                    global_base[i][j] = 1  # free space
    # Mark the goal on the global base map
    gx, gy = robot.goal
    goal_i, goal_j = gx // block_h, gy // block_w
    if 0 <= goal_i < downsampled_h and 0 <= goal_j < downsampled_w:
        global_base[goal_i][goal_j] = 5
    img_global = ax_global.imshow(global_base, cmap=cmap, vmin=0, vmax=5)

    # Simulation loop
    steps = 0
    while True:
        # Plan path based on current *known* information (exploration context)
        path = robot.plan_path()

        # Update local view centered around robot
        rx, ry = robot.pos
        local_x_min = max(0, rx - local_size//2); local_y_min = max(0, ry - local_size//2)
        local_x_max = min(maze.height, local_x_min + local_size); local_y_max = min(maze.width, local_y_min + local_size)
        if local_x_max - local_x_min < local_size:
            local_x_min = max(0, local_x_max - local_size)
        if local_y_max - local_y_min < local_size:
            local_y_min = max(0, local_y_max - local_size)
        local_h = local_x_max - local_x_min
        local_w = local_y_max - local_y_min
        local_display = [[0]*local_w for _ in range(local_h)]
        for i in range(local_x_min, local_x_max):
            for j in range(local_y_min, local_y_max):
                if robot.known_map[i][j] == -1:
                    local_display[i - local_x_min][j - local_y_min] = 0  # unknown
                elif robot.known_map[i][j] == 0:
                    # known free; mark visited cells in light blue
                    local_display[i - local_x_min][j - local_y_min] = 3 if (i, j) in robot.visited_path else 1
                else:
                    local_display[i - local_x_min][j - local_y_min] = 2  # wall
        # Mark robot and (if visible in window) goal in local view
        local_rx = rx - local_x_min; local_ry = ry - local_y_min
        local_display[local_rx][local_ry] = 4  # robot
        if local_x_min <= gx < local_x_max and local_y_min <= gy < local_y_max:
            local_display[gx - local_x_min][gy - local_y_min] = 5  # goal

        # Update global view: start with base and overlay robot's position
        global_display = [list(row) for row in global_base]  # copy base map
        rb_i = rx // block_h; rb_j = ry // block_w
        if 0 <= rb_i < downsampled_h and 0 <= rb_j < downsampled_w:
            global_display[rb_i][rb_j] = 4  # mark robot on global map

        # Refresh plots with new data
        img_local.set_data(local_display)
        img_global.set_data(global_display)
        fig_local.canvas.draw()
        fig_global.canvas.draw()
        
        plt.pause(0.001)  # small delay for animation effect

        # Termination conditions
        if path is None:
            print("No path to goal could be found with current knowledge. Exploration ended.")
            break
        if robot.pos == robot.goal:
            print(f"Goal reached in {steps} steps!")
            break

        # Move the robot one step along the planned path (simulate movement)
        robot.move_one_step(path)
        steps += 1

    # Keep the final plots open
    plt.ioff()
    plt.show()

if __name__ == '__main__':
    run_simulation()

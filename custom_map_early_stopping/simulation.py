import numpy as np
import matplotlib.pyplot as plt
from maze_generator import generate_maze
from robot import Robot
import sys

def load_custom_map(map_file):
    data = np.load(map_file)
    return data['grid'], data['start'], data['goal']

use_custom_map = True
map_file_name = "./custom_map.npz"

# Check for custom map argument
if use_custom_map:
    maze, start, goal = load_custom_map(map_file_name)
    start = (start[0], start[1])
    goal = (goal[0], goal[1])
    GRID_SIZE = maze.shape[0]
else:
    from maze_generator import generate_maze
    GRID_SIZE = 1000
    maze = generate_maze(GRID_SIZE)
    start = (0, 0)
    goal = (GRID_SIZE-1, GRID_SIZE-1)

robot = Robot(start, goal, map_size=GRID_SIZE, vision_radius=50)
robot.known_map = np.copy(maze)

# GRID_SIZE = 10000
# maze = generate_maze(GRID_SIZE)
# start = (0, 0)
# goal = (GRID_SIZE-1, GRID_SIZE-1)
robot = Robot(start, goal, map_size=GRID_SIZE, vision_radius=50)

# Visualization setup
fig, (ax_local, ax_global) = plt.subplots(1, 2, figsize=(12, 6))
plt.ion()

# Initial sensing
robot.sense(maze)

def get_local_area(robot, known_map):
    """Get properly aligned local area around robot"""
    x, y = robot.current_position
    vr = robot.vision_radius

    # Calculate bounds in matrix coordinates (rows, columns)
    row_min = max(0, y - vr)
    row_max = min(GRID_SIZE-1, y + vr)
    col_min = max(0, x - vr)
    col_max = min(GRID_SIZE-1, x + vr)

    return known_map[row_min:row_max+1, col_min:col_max+1]

# Initialize local view
local_area = get_local_area(robot, robot.known_map)
local_display = np.zeros_like(local_area, dtype=float)
local_display[local_area == 0] = 1.0  # Free (white)
local_display[local_area == 1] = 0.0  # Obstacle (black)
local_display[local_area == -1] = 0.5  # Unknown (gray)
im_local = ax_local.imshow(local_display, cmap='gray', vmin=0, vmax=1)
ax_local.set_title("Local View")

# Initialize robot marker
robot_marker, = ax_local.plot([robot.vision_radius], [robot.vision_radius], 
                             'ro', markersize=8, markeredgewidth=2)

# Initialize path overlay
path_overlay = np.zeros((*local_display.shape, 4))
path_image = ax_local.imshow(path_overlay, interpolation='none')

# Global view setup
block_size = 10
maze_float = maze.astype(float)
maze_blocks = maze_float.reshape(GRID_SIZE//block_size, block_size, 
                                 GRID_SIZE//block_size, block_size)
obstacle_frac = maze_blocks.mean(axis=(1,3))
im_global = ax_global.imshow(obstacle_frac, cmap='gray', vmin=0, vmax=1)
global_robot_marker, = ax_global.plot([0], [0], 'ro', markersize=6)
ax_global.set_title("Global Overview")

plt.tight_layout()
plt.show()

step = 0
while robot.current_position != robot.goal:

    print(robot.current_position)
    print(goal)
    step += 1

    # 1. Sense environment first
    robot.sense(maze)

    # 2. Plan path
    path = robot.plan_path()

    print(path)
    # 3. Move if valid path exists
    if path and len(path) > 1:
        robot.move_one_step(path)

    # Update local visualization
    local_area = get_local_area(robot, robot.known_map)

    # Create display matrix with correct color mapping
    local_display = np.zeros_like(local_area, dtype=float)
    local_display[local_area == 0] = 1.0  # Free (white)
    local_display[local_area == 1] = 0.0  # Obstacle (black)
    local_display[local_area == -1] = 0.5  # Unknown (gray)

    # Update local view image
    im_local.set_data(local_display)

    # Update path overlay
    path_overlay = np.zeros((*local_area.shape, 4))
    x, y = robot.current_position
    vr = robot.vision_radius
    for (px, py) in robot.visited_path:
        row = py - (y - vr) if (y - vr) > 0 else py
        col = px - (x - vr) if (x - vr) > 0 else px
        if 0 <= row < path_overlay.shape[0] and 0 <= col < path_overlay.shape[1]:
            path_overlay[row, col] = [0, 1, 0, 0.3]  # Semi-transparent green
    path_image.set_data(path_overlay)

    # Update robot marker position (center of local view)
    robot_marker.set_xdata([robot.vision_radius])
    robot_marker.set_ydata([robot.vision_radius])

    # Update global view
    global_x = robot.current_position[0] // block_size
    global_y = robot.current_position[1] // block_size
    global_robot_marker.set_xdata([global_x])
    global_robot_marker.set_ydata([global_y])

    # Update titles
    ax_local.set_title(f"Local View - Step {step}")
    ax_global.set_title(f"Global Overview - Step {step}")

    try:
        plt.pause(0.001)
        fig.canvas.flush_events()
    except Exception as e:
        print(f"Visualization error: {str(e)}")
        exit()

# Final visualization
ax_local.set_title(f"Goal Reached in {step} Steps!")
ax_global.set_title("Final Overview")
plt.ioff()
plt.show()
print(f"Navigation complete! Steps taken: {step}, Path length: {len(robot.visited_path)}")
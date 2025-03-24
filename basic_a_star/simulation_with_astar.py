"""
Simulation script: runs the robot in a maze with visualization.
"""
import time
import matplotlib.pyplot as plt
from maze_generator import Maze
from robot import Robot

def run_simulation():
    # Create a random maze and initialize the robot
    maze = Maze(50, 50)
    robot = Robot(maze, start=(0,0), goal=(maze.height-1, maze.width-1), vision_radius=5)
    
    # Setup the plot for visualization
    plt.ion()  # interactive mode on for real-time updates
    fig, ax = plt.subplots(figsize=(6,6))
    # Define colormap for different cell states in the visualization
    # 0: unknown (gray), 1: known free (white), 2: wall (black),
    # 3: visited path (light blue), 4: robot (red), 5: goal (green)
    from matplotlib.colors import ListedColormap
    cmap = ListedColormap([
        (0.7, 0.7, 0.7),  # unknown
        (1.0, 1.0, 1.0),  # known free
        (0.0, 0.0, 0.0),  # wall
        (0.4, 0.8, 1.0),  # visited path
        (1.0, 0.0, 0.0),  # robot
        (0.0, 1.0, 0.0)   # goal
    ])
    # Initialize the display with all unknown cells
    img = ax.imshow([[0]*maze.width for _ in range(maze.height)], cmap=cmap, vmin=0, vmax=5)
    ax.set_title("Omnidirectional Robot Maze Exploration")
    ax.set_xticks([]); ax.set_yticks([])  # Hide axis ticks for clarity
    
    # Simulation loop
    steps = 0
    while True:
        # Plan path based on current knowledge
        path = robot.plan_path()
        # Prepare a grid for visualization of known cells
        display_grid = [[0]*maze.width for _ in range(maze.height)]
        for i in range(maze.height):
            for j in range(maze.width):
                if robot.known_map[i][j] == -1:
                    display_grid[i][j] = 0   # unknown
                elif robot.known_map[i][j] == 0:
                    # Free space (known); mark if visited or not
                    display_grid[i][j] = 3 if (i, j) in robot.visited_path else 1
                else:
                    display_grid[i][j] = 2   # wall
        # Mark the robot and goal positions
        rx, ry = robot.pos
        gx, gy = robot.goal
        display_grid[rx][ry] = 4   # robot's current position
        if (rx, ry) != (gx, gy):
            display_grid[gx][gy] = 5  # goal (only mark if robot hasn't reached it yet)
        
        # Update the plot with the new state
        img.set_data(display_grid)
        fig.canvas.draw()
        plt.pause(0.2)  # brief pause to animate the movement
        
        # Check termination conditions
        if path is None:
            print("No path to goal could be found with current knowledge. Exploration ended.")
            break
        if robot.pos == robot.goal:
            print(f"Goal reached in {steps} steps!")
            break
        
        # Move the robot one step along the planned path
        robot.move_one_step(path)
        steps += 1
    
    # Keep the final plot open
    plt.ioff()
    plt.show()

if __name__ == '__main__':
    run_simulation()

# Omnidirectional Robot Maze Exploration

This project simulates an omnidirectional robot navigating a 20x20 grid maze using dynamic path planning with limited vision.

## Files
- **maze_generator.py**: Generates a random 20x20 maze (grid) with walls.
- **path_planner.py**: Implements an A* pathfinding algorithm for the robot.
- **robot.py**: Defines the Robot class (with limited sensor vision and path planning).
- **simulation.py**: Main script to run the simulation and visualize the robot's exploration.

## Requirements
- Python 3.x
- Matplotlib (for visualization)

## How to Run
1. Ensure all the above files are in the same directory.
2. Open a terminal in that directory.
3. Run the simulation using: python simulation.py
4. A window will open showing the maze. The robot (red square) will start at the top-left and attempt to reach the bottom-right goal (green square).
5. Walls are shown in black, unknown areas in gray, known free spaces in white, and the robot's visited path in light blue.

The robot will explore and update its path in real-time as it discovers new walls. The simulation will display the robot's movement step by step until it reaches the goal.
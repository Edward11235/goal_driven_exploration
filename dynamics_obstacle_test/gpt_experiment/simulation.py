
import matplotlib.pyplot as plt
import numpy as np
from simple_astar import astar
from obstacle_tracking import track_and_predict
from obstacle_extraction import extract_obstacles
import time

grid_size = 20
ego_start, ego_goal = (0, 0), (19, 19)

ego_pos = ego_start
obs_path = [(7,7), (7,13), (13,13), (13,7)]  # square path
obs_idx = 0
obs_pos = obs_path[obs_idx]
prev_obs_pos = obs_pos

occupancy_grid = np.zeros((grid_size, grid_size), dtype=int)

plt.ion()
fig, ax = plt.subplots(figsize=(6, 6))

for step in range(100):
    ax.clear()
    ax.set_xlim(-1, grid_size)
    ax.set_ylim(-1, grid_size)
    ax.set_xticks(range(grid_size))
    ax.set_yticks(range(grid_size))
    ax.grid(True)
    ax.set_title(f"Step {step}")

    # Move obstacle along the square path
    next_obs_idx = (obs_idx + 1) % len(obs_path)
    target_obs_pos = obs_path[next_obs_idx]
    if obs_pos == target_obs_pos:
        obs_idx = next_obs_idx
        target_obs_pos = obs_path[(obs_idx + 1) % len(obs_path)]

    move_vector = np.sign(np.array(target_obs_pos) - np.array(obs_pos))
    obs_pos = tuple((np.array(obs_pos) + move_vector).tolist())

    # Update occupancy grid without knowing ground truth explicitly (simulate sensing)
    occupancy_grid.fill(0)
    occupancy_grid[obs_pos] = 1  # obstacle appears as occupied

    # Extract obstacle position via CCL (sensor simulation)
    detected_obs = extract_obstacles(occupancy_grid)
    if detected_obs:
        curr_obs_pos = detected_obs[0]
        predicted_obs_pos = track_and_predict(prev_obs_pos, curr_obs_pos)
        prev_obs_pos = curr_obs_pos
    else:
        predicted_obs_pos = obs_pos

    # Mark predicted obstacle position as obstacle for path planning
    obstacles = {tuple(predicted_obs_pos)}
    path = astar(ego_pos, ego_goal, obstacles, grid_size)

    if path and len(path) > 1:
        ego_pos = path[1]  # move ego

    # Swap ego start and goal after reaching
    if ego_pos == ego_goal:
        ego_start, ego_goal = ego_goal, ego_start

    # Plot
    ax.plot(ego_pos[1], ego_pos[0], "bo", markersize=12, label="Ego Vehicle")
    ax.plot(obs_pos[1], obs_pos[0], "ro", markersize=12, label="Obstacle (Ground Truth)")
    ax.plot(predicted_obs_pos[1], predicted_obs_pos[0], "rx", markersize=12, label="Predicted Obstacle")
    ax.plot(ego_goal[1], ego_goal[0], "gx", markersize=12, label="Goal")
    ax.legend(loc="upper right")
    plt.pause(0.2)

plt.ioff()
plt.show()

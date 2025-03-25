
from scipy.ndimage import label, center_of_mass

def extract_obstacles(occupancy_grid):
    labeled_grid, num_features = label(occupancy_grid)
    centroids = center_of_mass(occupancy_grid, labeled_grid, range(1, num_features + 1))
    centroids = [(int(c[0]), int(c[1])) for c in centroids]
    return centroids

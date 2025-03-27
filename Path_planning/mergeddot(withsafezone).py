import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

# Load CSV files
csv_file1 = 'lidar_positions1.csv'
csv_file2 = 'lidar_positions2.csv'
csv_file3 = 'lidar_positions3.csv'

data1 = pd.read_csv(csv_file1)[['x', 'y']].values
data2 = pd.read_csv(csv_file2)[['x', 'y']].values
data3 = pd.read_csv(csv_file3)[['x', 'y']].values

# Function to apply transformation: translation and rotation
def transform_points(data, translation, rotation_angle):
    theta = np.radians(rotation_angle)
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    transformed_points = np.dot(data, rotation_matrix) + translation
    return transformed_points

# Apply fixed rotations and translations
rotation_angle1 = -70
rotation_angle2 = -39
rotation_angle3 = 50

translation1 = [-0.5, -0.5]
translation2 = [-0.4, 0.5]
translation3 = [0.9, -0.5]

data1_transformed = transform_points(data1, translation1, rotation_angle1)
data2_transformed = transform_points(data2, translation2, rotation_angle2)
data3_transformed = transform_points(data3, translation3, rotation_angle3)

# Combine all transformed data
merged_data = np.vstack((data1_transformed, data2_transformed, data3_transformed))

# Function to create a buffer zone around points
def create_safe_zone(points, buffer_radius=0.3, num_points=100):
    safe_zone_points = []
    for point in points:
        for _ in range(num_points):
            angle = np.random.uniform(0, 2 * np.pi)
            distance = np.random.uniform(0, buffer_radius)
            dx = distance * np.cos(angle)
            dy = distance * np.sin(angle)
            safe_zone_points.append([point[0] + dx, point[1] + dy])
    return np.array(safe_zone_points)

# Create a safe zone around the merged points (buffer of 30 cm)
safe_zone_radius = 0.15  # 30 cm
safe_zone_points = create_safe_zone(merged_data, buffer_radius=safe_zone_radius)

# Grid map settings
cell_size = 0.03  # Size of each grid cell (3 cm)

# Determine the grid boundaries
x_min, x_max = safe_zone_points[:, 0].min() - cell_size, safe_zone_points[:, 0].max() + cell_size
y_min, y_max = safe_zone_points[:, 1].min() - cell_size, safe_zone_points[:, 1].max() + cell_size

# Create grid
x_bins = np.arange(x_min, x_max, cell_size)
y_bins = np.arange(y_min, y_max, cell_size)
grid = np.zeros((len(y_bins), len(x_bins)), dtype=int)

# Map points to grid cells
for x, y in safe_zone_points:
    x_idx = np.digitize(x, x_bins) - 1
    y_idx = np.digitize(y, y_bins) - 1
    if 0 <= x_idx < len(x_bins) and 0 <= y_idx < len(y_bins):
        grid[y_idx, x_idx] = 1

# Plot the grid map
plt.figure(figsize=(10, 10))
plt.imshow(grid, extent=[x_min, x_max, y_min, y_max], origin='lower', cmap='gray_r', interpolation='nearest')

# Add labels and grid lines
plt.title("Grid Map with Safe Zone Around Obstacles (30 cm Buffer)")
plt.xlabel("X Coordinate (meters)")
plt.ylabel("Y Coordinate (meters)")
plt.grid(which='both', color='lightgray', linestyle='--', linewidth=0.5)

# Ensure equal scaling on both axes
plt.gca().set_aspect('equal', adjustable='box')

plt.show()

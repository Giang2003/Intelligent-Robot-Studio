import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import heapq

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
def create_safe_zone(points, buffer_radius=0.15, num_points=100):
    safe_zone_points = []
    for point in points:
        for _ in range(num_points):
            angle = np.random.uniform(0, 2 * np.pi)
            distance = np.random.uniform(0, buffer_radius)
            dx = distance * np.cos(angle)
            dy = distance * np.sin(angle)
            safe_zone_points.append([point[0] + dx, point[1] + dy])
    return np.array(safe_zone_points)

# Create safe zone
safe_zone_radius = 0.15
safe_zone_points = create_safe_zone(merged_data, buffer_radius=safe_zone_radius)

# Grid map settings
cell_size = 0.03

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

# Heuristic function for A* (Euclidean distance)
def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

# A* algorithm with diagonal movement
def astar_search(start, goal, grid, grid_range, cell_size):
    grid_size = grid.shape
    start_cell = ((start[0] - grid_range[0]) / cell_size, (start[1] - grid_range[1]) / cell_size)
    goal_cell = ((goal[0] - grid_range[0]) / cell_size, (goal[1] - grid_range[1]) / cell_size)
    start_cell = (int(start_cell[0]), int(start_cell[1]))
    goal_cell = (int(goal_cell[0]), int(goal_cell[1]))

    # Priority queue
    open_set = []
    heapq.heappush(open_set, (0, start_cell))
    came_from = {}
    g_score = {start_cell: 0}
    f_score = {start_cell: heuristic(start_cell, goal_cell)}

    # Directions including diagonals
    directions = [
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1)
    ]

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal_cell:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_cell)
            return path[::-1]

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < grid.shape[1] and 0 <= neighbor[1] < grid.shape[0]:
                if grid[neighbor[1], neighbor[0]] == 1:
                    continue

                tentative_g_score = g_score[current] + (np.sqrt(2) if dx != 0 and dy != 0 else 1)

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal_cell)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None

# Set start and goal points
start = (0.5, -0.5)
goal = (-0.5, 0.5)

# Find path using A*
path = astar_search(start, goal, grid, (x_min, y_min), cell_size)

# Plot the grid map with path
plt.figure(figsize=(10, 10))
plt.imshow(grid, extent=[x_min, x_max, y_min, y_max], origin='lower', cmap='gray_r', interpolation='nearest')

# Plot start and goal points
plt.scatter(start[0], start[1], color='blue', s=50, label='Start')
plt.scatter(goal[0], goal[1], color='green', s=50, label='Goal')

# Plot the path in red
if path:
    path_coords = [(x_min + cell[0] * cell_size + cell_size / 2, y_min + cell[1] * cell_size + cell_size / 2) for cell in path]
    path_x, path_y = zip(*path_coords)
    plt.plot(path_x, path_y, color='red', linewidth=2, marker='o', markersize=5, label='Path')
else:
    print("No path found.")

# Plot settings
plt.title("Grid Map with Safe Zone and A* Pathfinding")
plt.xlabel("X Coordinate (meters)")
plt.ylabel("Y Coordinate (meters)")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()

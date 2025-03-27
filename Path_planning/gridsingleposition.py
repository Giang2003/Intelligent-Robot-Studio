import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load CSV file
csv_file = 'lidar_positions3.csv'  # Replace with your file path
data = pd.read_csv(csv_file)

# Function to rotate coordinates by a given angle (in degrees)
def rotate_coordinates(x, y, angle_deg):
    angle_rad = np.radians(angle_deg)
    x_rot = x * np.cos(angle_rad) - y * np.sin(angle_rad)
    y_rot = x * np.sin(angle_rad) + y * np.cos(angle_rad)
    return x_rot, y_rot

# Check if the file has the expected columns
if 'x' in data.columns and 'y' in data.columns:
    # Rotate the lidar points by 10 degrees
    angle_deg = 0
    data['x_rot'], data['y_rot'] = rotate_coordinates(data['x'], data['y'], angle_deg)

    # Plot the lidar points (rotated)
    plt.figure(figsize=(10, 8))
    plt.subplot(1, 2, 1)  # Set up a 1x2 grid of subplots
    
    # Scatter plot of rotated lidar points
    plt.scatter(data['x_rot'], data['y_rot'], color='blue', s=5, label='Rotated Lidar Points')
    plt.title(f"Lidar Map for Right position)")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    
    # Create a grid map for rotated points
    grid_resolution = 0.2
    x_min, y_min = data[['x_rot', 'y_rot']].min()
    x_max, y_max = data[['x_rot', 'y_rot']].max()
    
    # Calculate grid size
    x_size = int((x_max - x_min) / grid_resolution) + 1
    y_size = int((y_max - y_min) / grid_resolution) + 1
    
    # Initialize a binary grid map
    grid_map = np.zeros((x_size, y_size), dtype=int)
    
    # Map rotated lidar points to the grid
    for x, y in zip(data['x_rot'], data['y_rot']):
        grid_x = int((x - x_min) / grid_resolution)
        grid_y = int((y - y_min) / grid_resolution)
        grid_map[grid_x, grid_y] = 1  # Mark cell as occupied

    # Plot the grid map (rotated)
    # plt.subplot(1, 2, 2)
    # plt.imshow(grid_map.T, origin='lower', extent=[x_min, x_max, y_min, y_max], cmap='Greys', alpha=0.7)
    # plt.title(f"Grid Map for Right position, Resolution: 0.2)")
    # plt.xlabel("X Coordinate")
    # plt.ylabel("Y Coordinate")
    # plt.grid(True)

    plt.tight_layout()
    plt.show()
    


import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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

# Apply fixed rotations
rotation_angle1 = -70 
rotation_angle2 = -39  
rotation_angle3 = 50   

translation1 = [-0.5, -0.5] 
translation2 = [-0.4, 0.5]   
translation3 = [0.9, -0.5]   

# Transform the datasets with fixed rotations
data1_transformed = transform_points(data1, translation1, rotation_angle1)
data2_transformed = transform_points(data2, translation2, rotation_angle2)
data3_transformed = transform_points(data3, translation3, rotation_angle3)

# Combine all transformed data into a single dataset
merged_data = np.vstack((data1_transformed, data2_transformed, data3_transformed))

# Plot the dot map
plt.figure(figsize=(10, 10))
plt.scatter(data1_transformed[:, 0], data1_transformed[:, 1], color='red', s=5, label='Lidar Position 1')
plt.scatter(data2_transformed[:, 0], data2_transformed[:, 1], color='green', s=5, label='Lidar Position 2')
plt.scatter(data3_transformed[:, 0], data3_transformed[:, 1], color='purple', s=5, label='Lidar Position 3')
plt.scatter(merged_data[:, 0], merged_data[:, 1], color='blue', s=2, label='Merged Points')

# Plot settings
plt.title("Dot Map of Merged Lidar Scans with Fixed Rotations")
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.legend()
plt.grid(True)
plt.axis('equal')  # Ensure equal scaling on both axes
plt.show()

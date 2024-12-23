import numpy as np
import matplotlib.pyplot as plt
import yaml
from skimage import io, feature
from rdp import rdp  # Install with: pip install rdp

# Step 1: Load the .pgm map and .yaml data
pgm_file = 'my_map.pgm'
yaml_file = 'my_map.yaml'

# Load the map image
map_image = io.imread(pgm_file)

# Load the YAML data for origin and resolution
with open(yaml_file, 'r') as file:
    map_metadata = yaml.safe_load(file)
origin = map_metadata['origin']
resolution = map_metadata['resolution']

# Step 2: Preprocess the map to binary
# Step 2: Preprocess the map to binary and convert to 8-bit
binary_map = np.where(map_image < 128, 0, 1).astype(np.uint8)  # Convert to 8-bit integer
  # Binary map (0 for obstacles, 1 for free space)

# Step 3: Edge detection using Canny
edges = feature.canny(binary_map, sigma=2)

# Extract coordinates of edges
edge_points = np.column_stack(np.nonzero(edges))

# Step 4: Apply Ramer–Douglas–Peucker (RDP) algorithm
tolerance = 2.0  # Adjust based on required accuracy
simplified_points = rdp(edge_points, epsilon=tolerance)

# Step 5: Convert coordinates to map’s world frame
world_lines = []
for point in simplified_points:
    x_world = origin[0] + point[1] * resolution
    y_world = origin[1] + (binary_map.shape[0] - point[0]) * resolution
    world_lines.append([x_world, y_world])  # Ensure this is a 2D list of points

# Convert to numpy array if not empty
if world_lines:
    world_lines = np.array(world_lines)
else:
    print("No line segments detected.")
    world_lines = np.empty((0, 2))  # Create an empty array with shape (0, 2) for compatibility

# Step 6: Plotting
plt.imshow(binary_map, cmap='gray', origin='lower', extent=[
    origin[0], origin[0] + binary_map.shape[1] * resolution,
    origin[1], origin[1] + binary_map.shape[0] * resolution
])
if world_lines.size > 0:  # Only plot if world_lines has data
    plt.plot(world_lines[:, 0], world_lines[:, 1], 'r-', linewidth=1.5, label='Extracted Lines')
plt.legend()
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.title('Line Extraction using RDP Algorithm')
plt.show()



import yaml
import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the .yaml file
yaml_file_path = 'my_map.yaml'
with open(yaml_file_path, 'r') as file:
    map_metadata = yaml.safe_load(file)

# Extract data from the YAML
resolution = map_metadata['resolution']  # meters per pixel
origin = map_metadata['origin']  # [x, y, theta] in meters
pgm_file_path = map_metadata['image']  # path to the .pgm file

# Load the .pgm map image
map_image = cv2.imread(pgm_file_path, cv2.IMREAD_GRAYSCALE)

# Threshold to find occupied areas (walls)
occupied_thresh = map_metadata.get('occupied_thresh', 0.65) * 255  # Convert to grayscale range
walls = (map_image < occupied_thresh).astype(np.uint8)  # 1 for occupied (walls), 0 for free space

# Function to convert pixel coordinates to map frame
def pixel_to_map(x_pixel, y_pixel, origin, resolution, height):
    x_map = origin[0] + (x_pixel * resolution)
    y_map = origin[1] + ((height - y_pixel) * resolution)  # Invert y-axis
    return x_map, y_map

# Get the image dimensions
height, width = map_image.shape

# Find the pixel locations of the occupied areas (walls)
occupied_pixels = np.argwhere(walls == 1)  # Get (y, x) coordinates of occupied pixels

# Convert pixel locations to map frame coordinates
map_wall_locations = []

for y_pixel, x_pixel in occupied_pixels:
    x_map, y_map = pixel_to_map(x_pixel, y_pixel, origin, resolution, height)
    map_wall_locations.append((x_map, y_map))

# Separate x and y for plotting
x_coords = [coord[0] for coord in map_wall_locations]
y_coords = [coord[1] for coord in map_wall_locations]

# Set up plot
fig, ax = plt.subplots(figsize=(10, 10))
ax.scatter(x_coords, y_coords, s=1, color='black', label="Occupied Areas (Walls)")
ax.set_title('Click to Annotate Wall Start/End Points')
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.legend()
ax.grid(True)
ax.axis('equal')  # Equal scaling for both axes

# List to store annotated start and end points
points = []

# Function to handle mouse clicks
def onclick(event):
    # Capture only clicks inside the plot area
    if event.xdata is not None and event.ydata is not None:
        # Append the clicked point to the list
        points.append((event.xdata, event.ydata))
        # Mark the point on the plot
        ax.plot(event.xdata, event.ydata, 'ro')  # Red circle for clicked points
        plt.draw()  # Update the plot

        # If two points are selected, draw a line segment
        if len(points) % 2 == 0:
            # Draw a line between the last two points
            ax.plot([points[-2][0], points[-1][0]], [points[-2][1], points[-1][1]], 'b-')  # Blue line
            plt.draw()

# Function to save points to a file
def save_points_to_file(filename='my_map.txt'):
    with open(filename, 'w') as f:
        for point in points:
            f.write(f"{point[0]}, {point[1]}\n")
    print(f"Annotated points saved to {filename}")

# Connect the click event to the onclick function
cid = fig.canvas.mpl_connect('button_press_event', onclick)

# Optionally, connect a key press to save points
def on_key(event):
    if event.key == 's':  # Press 's' to save points
        save_points_to_file()

fig.canvas.mpl_connect('key_press_event', on_key)

plt.show()
print(points)


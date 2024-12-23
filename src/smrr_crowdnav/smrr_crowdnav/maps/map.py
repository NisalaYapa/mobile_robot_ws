import cv2
import numpy as np
import yaml
import matplotlib.pyplot as plt

#### This is for annotate the given map (using .pgm and .yaml) to represent obstacles as lines.
### After running this code, it will appear a plot with dots
### first touch - stating point of the line
### second touch - end of the line
### to save the lines press "s". this will update the .txt file (to use later in combined_lines_publisher.py)


# Load the map image
map_image = cv2.imread('my_map.pgm', cv2.IMREAD_GRAYSCALE)

# Read the .yaml file
with open('my_map.yaml', 'r') as file:
    map_metadata = yaml.safe_load(file)

# Extract necessary parameters
resolution = map_metadata['resolution']  # meters per pixel
origin = map_metadata['origin']          # (x, y, yaw)

# Threshold the image to make walls distinct
_, thresh_image = cv2.threshold(map_image, 127, 255, cv2.THRESH_BINARY_INV)

# Use Canny edge detection to find edges
edges = cv2.Canny(thresh_image, 50, 150)

# Find contours in the edge-detected image
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Check if contours were found
if not contours:
    print("No contours found.")
else:
    print(f"Found {len(contours)} contours.")

# Initialize a list to store line segments
line_segments = []

# Convert pixel positions to real-world coordinates
def pixel_to_real_world(pixel_point):
    x_pixel, y_pixel = pixel_point
    x_meters = origin[0] + x_pixel * resolution
    y_meters = origin[1] + (map_image.shape[0] - y_pixel) * resolution  # Invert y-axis
    return (x_meters, y_meters)

# Ramer-Douglas-Peucker algorithm implementation
def rdp(points, epsilon):
    if len(points) < 3:
        return points
    
    # Find the point with the maximum distance from the line between the endpoints
    start, end = points[0], points[-1]
    dmax = 0
    index = -1
    for i in range(1, len(points) - 1):
        d = np.abs(np.cross(end - start, start - points[i])) / np.linalg.norm(end - start)
        if d > dmax:
            index = i
            dmax = d
    
    if dmax > epsilon:
        left = rdp(points[:index + 1], epsilon)
        right = rdp(points[index:], epsilon)
        return left[:-1] + right
    else:
        return [start, end]

# Approximate each contour to a polyline (line segments)
for contour in contours:
    epsilon = 5  # Adjust this value for how much simplification is needed
    approx = cv2.approxPolyDP(contour, epsilon, True)

    # Convert to real-world coordinates and apply RDP for simplification
    points = np.array([pixel_to_real_world(tuple(pt[0])) for pt in approx])
    simplified_points = rdp(points, epsilon=0.1)  # Adjust epsilon as needed

    # Store the line segments
    for i in range(len(simplified_points) - 1):
        line_segments.append((simplified_points[i], simplified_points[i + 1]))

# Check if line segments were found
if not line_segments:
    print("No line segments found.")
else:
    print(f"Found {len(line_segments)} line segments.")

# Visualization using Matplotlib
plt.figure(figsize=(10, 10))
plt.imshow(map_image, cmap='gray', origin='upper')
plt.title('Map with Detected Line Segments and Contours')

# Draw each line segment on the map
for start, end in line_segments:
    plt.plot([start[0], end[0]], [start[1], end[1]], color='red')

# Draw each contour in a different color
for contour in contours:
    contour_points = np.array([pixel_to_real_world(tuple(pt[0])) for pt in contour])
    plt.plot(contour_points[:, 0], contour_points[:, 1], color='blue', alpha=0.5)  # Change color and transparency as needed

# Set the axes to be equal and adjust limits
plt.axis('equal')
plt.xlim(origin[0] - 1, origin[0] + map_image.shape[1] * resolution + 1)
plt.ylim(origin[1] - 1, origin[1] + map_image.shape[0] * resolution + 1)
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.grid()
plt.show()


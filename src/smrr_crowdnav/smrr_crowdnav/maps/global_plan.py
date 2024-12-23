import numpy as np
import cv2
import matplotlib.pyplot as plt
import yaml
from queue import PriorityQueue

# Load the map (grayscale .pgm) and metadata (.yaml)
def load_map(pgm_file, yaml_file):
    map_image = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)

    with open(yaml_file, 'r') as file:
        yaml_data = yaml.safe_load(file)
        resolution = yaml_data['resolution']
        origin = yaml_data['origin']

    return map_image, resolution, origin

# Convert world pose (x, y) to grid map coordinates
def pose_to_grid(pose, resolution, origin):
    x, y = pose
    grid_x = int((x - origin[0]) / resolution)
    grid_y = int((y - origin[1]) / resolution)
    return grid_x, grid_y

# Heuristic function for A* (Euclidean distance)
def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

# A* algorithm for shortest pathfinding
def a_star(occupancy_grid, start, goal):
    # The size of the map
    height, width = occupancy_grid.shape

    # Priority queue to store the frontier nodes
    frontier = PriorityQueue()
    frontier.put((0, start))  # (priority, node)
    
    # Dictionaries to store the cost and path
    came_from = {}
    cost_so_far = {}

    came_from[start] = None
    cost_so_far[start] = 0

    # A* search
    while not frontier.empty():
        _, current = frontier.get()

        # If the goal is reached, reconstruct the path
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        # Neighbor movements: Up, Down, Left, Right, and Diagonals
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)

            # Check if the neighbor is within bounds
            if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height:
                # Check if the neighbor is an obstacle (black pixel)
                if occupancy_grid[neighbor[1], neighbor[0]] <= 250:  # 0 = black = obstacle
                    continue

                # Calculate the new cost to move to the neighbor
                new_cost = cost_so_far[current] + np.linalg.norm([dx, dy])

                # If the neighbor hasn't been visited or a cheaper path is found
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor)
                    frontier.put((priority, neighbor))
                    came_from[neighbor] = current

    # No path found
    return None

# Visualize the path on the map image
def visualize_path_on_map(map_image, path, start, goal, resolution, origin):
    # Convert to unsigned 8-bit if needed
    if map_image.dtype == np.int8:
        map_image = map_image.astype(np.uint8)
    
    # Convert to RGB to draw the path in color
    map_rgb = cv2.cvtColor(map_image, cv2.COLOR_GRAY2RGB)
    
    # Scale the path and start/goal positions to fit the map resolution and origin
    def grid_to_map_coordinates(grid_point):
        x, y = grid_point
        return int(x), int(map_rgb.shape[0] - y)  # Adjust for image coordinate system
    
    # Draw the path in red
    for i in range(len(path) - 1):
        start_point = grid_to_map_coordinates(path[i])
        end_point = grid_to_map_coordinates(path[i+1])
        cv2.line(map_rgb, start_point, end_point, (0, 0, 255), 1)  # Red path

    # Draw start and goal positions
    start_point = grid_to_map_coordinates(start)
    goal_point = grid_to_map_coordinates(goal)
    
    cv2.circle(map_rgb, start_point, 2, (0, 255, 0), -1)  # Start point in green
    cv2.circle(map_rgb, goal_point, 2, (255, 0, 0), -1)   # Goal point in blue

    # Show the image using matplotlib
    plt.imshow(cv2.cvtColor(map_rgb, cv2.COLOR_BGR2RGB))
    plt.title("Path Visualization on Map")
    plt.show()

# Main function
def global_planner_and_visualize_on_map(pgm_file, yaml_file, start_pose, goal_pose):
    # Load the map
    map_image, resolution, origin = load_map(pgm_file, yaml_file)
    
    # Convert start and goal poses to grid coordinates
    start_grid = pose_to_grid(start_pose, resolution, origin)
    goal_grid = pose_to_grid(goal_pose, resolution, origin)
    
    # Perform A* to find the path
    path = a_star(map_image, start_grid, goal_grid)
    
    if path:
        # Visualize the map and the path
        visualize_path_on_map(map_image, path, start_grid, goal_grid, resolution, origin)
    else:
        print("No path found")

# Example usage
if __name__ == "__main__":
    pgm_file = "my_map.pgm"
    yaml_file = "my_map.yaml"
    
    start_pose = (0.0, 0.0)  # Example start pose (x, y) in meters
    goal_pose = (3.8, -1.0)   # Example goal pose (x, y) in meters
    
    global_planner_and_visualize_on_map(pgm_file, yaml_file, start_pose, goal_pose)


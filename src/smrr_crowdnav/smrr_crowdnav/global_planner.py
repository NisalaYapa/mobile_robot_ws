import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
from queue import PriorityQueue
import os
import yaml
from smrr_interfaces.msg import Entities
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TwistStamped, Point
from geometry_msgs.msg import Pose
import tf_transformations
from .include.transform import GeometricTransformations
from tf_transformations import euler_from_quaternion

class GlobalPathPlanner(Node):
    def __init__(self):
        super().__init__('global_path_planner')

        #  Load the YAML config file
        package_path = os.path.dirname(__file__)  # Current file's directory
        config_path = os.path.join(package_path,'config', 'config.yaml')
        
        with open(config_path, 'r') as file:
            configs = yaml.safe_load(file)
        
        # Get parameters for this class
        node_name = "ControlNode"  # Define your node's name
        node_configs = configs.get(node_name, {})

        # Set class attributes for each parameter
        for key, value in node_configs.items():
            setattr(self, key, value)  # Dynamically add attributes

        # Log the loaded parameters
        self.int_goals = getattr(self, 'Intermediate_Goals', 0)  # Default to 1 if not defined
        self.get_logger().info(f"Loaded Intermediate_goals size: {self.int_goals}")
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        self.transform = GeometricTransformations(self)
        
        # self.initial_pose_sub = self.create_subscription(
        #     PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        

        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.robot_position_callback, 10)
        
        # Publisher
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.intermediate_goals_pub = self.create_publisher(Entities, '/int_goals', 10)
        self.intermediate_goals_marker= self.create_publisher(MarkerArray, '/smrr_crowdnav/global_path', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose_processed', 10)
        
        # Variables
        self.map_data = None
        self.map_info = None
        self.start_pose = None
        self.goal_pose = None

        
        self.get_logger().info("Global Path Planner ready. Set start pose and goal in RViz2")
    
    def map_callback(self, msg):
        self.map_data = msg.data
        self.map_info = msg.info
        self.get_logger().info("Map received")
    

    def robot_position_callback(self, msg):
        #self.get_logger().info('Robot Velocity Callback')
        linear_x = msg.twist.twist.linear.x
        transformation = self.transform.get_transform('map', 'base_link')

        if transformation is None:
            self.ready = False
            return
        else:
            self.ready = True

        quaternion = (transformation.rotation.x, transformation.rotation.y, transformation.rotation.z, transformation.rotation.w)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)


      
        pose = Pose()
        pose.position.x = transformation.translation.x
        pose.position.y = transformation.translation.y
        pose.position.z = transformation.translation.z
        pose.orientation.x = transformation.rotation.x
        pose.orientation.y = transformation.rotation.y
        pose.orientation.z = transformation.rotation.z
        pose.orientation.w = transformation.rotation.w

        self.start_pose = pose

            
    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info(f"Goal pose set: {self.goal_pose.position.x}, {self.goal_pose.position.y}")
        self.try_plan_path()
    
    def try_plan_path(self):
        if self.map_data is None:
            self.get_logger().warn("No map available yet")
            return
        
        if self.start_pose is None:
            self.get_logger().warn("No start pose set (use RViz2 '2D Pose Estimate')")
            return
            
        if self.goal_pose is None:
            self.get_logger().warn("No goal pose set (use RViz2 'Nav2 Goal')")
            return
            
        self.plan_path()
    
    def world_to_map(self, x, y):
        """Convert world coordinates to map coordinates"""
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        
        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        return mx, my
    
    def map_to_world(self, mx, my):
        """Convert map coordinates to world coordinates"""
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        
        x = mx * res + ox
        y = my * res + oy
        return x, y
    
    def is_cell_free(self, x, y):
        """Check if cell is free (0) and within bounds"""
        if x < 0 or y < 0 or x >= self.map_info.width or y >= self.map_info.height:
            return False
        idx = y * self.map_info.width + x
        return self.map_data[idx] == 0
    
    def dijkstra(self, start, goal):
        """Dijkstra's path planning algorithm"""
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1),  # 4-connected
                      (1, 1), (-1, -1), (1, -1), (-1, 1)]  # 8-connected
        
        open_set = PriorityQueue()
        open_set.put((0, start))
        
        came_from = {}
        cost_so_far = {start: 0}
        came_from[start] = None
        
        while not open_set.empty():
            _, current = open_set.get()
            
            if current == goal:
                break
            
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not self.is_cell_free(*neighbor):
                    continue
                
                new_cost = cost_so_far[current] + math.sqrt(dx**2 + dy**2)
                
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost
                    open_set.put((priority, neighbor))
                    came_from[neighbor] = current
        
        # Reconstruct path
        path = []
        current = goal
        
        while current != start:
            path.append(current)
            current = came_from.get(current, None)
            if current is None:
                self.get_logger().error("No path found!")
                return None
        
        path.append(start)
        path.reverse()
        return path

    # def a_star(self, start, goal):
    #     """A* path planning algorithm with heuristic"""
    #     # 8-connected grid directions
    #     directions = [(1, 0), (-1, 0), (0, 1), (0, -1),
    #                 (1, 1), (-1, -1), (1, -1), (-1, 1)]
        
    #     open_set = PriorityQueue()
    #     open_set.put((0, start))  # (f_score, position)
        
    #     came_from = {}
        
    #     # Cost from start along best known path
    #     g_score = {start: 0}
        
    #     # Estimated total cost from start to goal through y
    #     f_score = {start: self.heuristic(start, goal)}
        
    #     while not open_set.empty():
    #         _, current = open_set.get()
            
    #         if current == goal:
    #             break  # Reached goal
            
    #         for dx, dy in directions:
    #             neighbor = (current[0] + dx, current[1] + dy)
                
    #             # Skip if neighbor is not free or out of bounds
    #             if not self.is_cell_free(*neighbor):
    #                 continue
                
    #             # Diagonal moves cost sqrt(2), others cost 1
    #             move_cost = math.sqrt(dx**2 + dy**2)
                
    #             # Tentative g_score for this neighbor
    #             tentative_g_score = g_score[current] + move_cost
                
    #             if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
    #                 # This path to neighbor is better than any previous one
    #                 came_from[neighbor] = current
    #                 g_score[neighbor] = tentative_g_score
    #                 f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
    #                 open_set.put((f_score[neighbor], neighbor))
        
    #     # Reconstruct path
    #     path = []
    #     current = goal
        
    #     while current != start:
    #         path.append(current)
    #         current = came_from.get(current, None)
    #         if current is None:
    #             self.get_logger().error("No path found!")
    #             return None
        
    #     path.append(start)
    #     path.reverse()
    #     return path

    # def heuristic(self, a, b):
    #     """Euclidean distance heuristic for A*"""
    #     dx = a[0] - b[0]
    #     dy = a[1] - b[1]
    #     return math.sqrt(dx**2 + dy**2)


    def a_star(self, start, goal):
        """A* algorithm with increased costs near obstacles"""
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1),
                    (1, 1), (-1, -1), (1, -1), (-1, 1)]  # 8-connected

        open_set = PriorityQueue()
        open_set.put((0, start))
        
        came_from = {}
        g_score = {start: 0}  # Actual cost from start
        f_score = {start: self.heuristic(start, goal)}  # Estimated total cost
        
        # Obstacle cost parameters
        min_safe_distance = 0.2  # meters
        max_cost_distance = 0.5   # meters
        max_cost_multiplier = 5.0
        
        while not open_set.empty():
            _, current = open_set.get()
            
            if current == goal:
                break
                
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Skip if out of bounds or occupied
                if not self.is_cell_free(*neighbor):
                    continue
                
                # Base movement cost (diagonal vs straight)
                move_cost = math.sqrt(dx**2 + dy**2)
                
                # Get obstacle proximity cost multiplier
                obstacle_cost = self.get_obstacle_cost(*neighbor, 
                                                    min_safe_distance,
                                                    max_cost_distance,
                                                    max_cost_multiplier)
                
                # Apply cost multiplier
                weighted_move_cost = move_cost * obstacle_cost
                
                tentative_g_score = g_score[current] + weighted_move_cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))
        
        # Path reconstruction (same as before)
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from.get(current, None)
            if current is None:
                self.get_logger().error("No path found!")
                return None
        path.append(start)
        path.reverse()
        return path

    def get_obstacle_cost(self, x, y, min_safe_dist, max_cost_dist, max_multiplier):
        """Calculate cost multiplier based on proximity to obstacles"""
        if not hasattr(self, 'distance_field'):
            self.precompute_distance_field()
        
        # Get distance to nearest obstacle in meters
        resolution = self.map_info.resolution
        distance = self.distance_field[y][x] * resolution
        
        if distance < min_safe_dist:
            return float('inf')  # Completely blocked
        elif distance >= max_cost_dist:
            return 1.0  # No additional cost
        
        # Quadratic cost increase as we get closer to obstacles
        t = (max_cost_dist - distance) / (max_cost_dist - min_safe_dist)
        return 1.0 + (max_multiplier - 1.0) * t * t

    def precompute_distance_field(self):
        """Precompute distance to nearest obstacle for all cells""" 
        from scipy.ndimage import distance_transform_edt 
        
        width = self.map_info.width 
        height = self.map_info.height 
        
        # Create obstacle matrix (1 for free, 0 for obstacle)
        obstacle_grid = np.ones((height, width)) 
        for y in range(height):
            for x in range(width):
                if self.map_data[y * width + x] > 0:  # Occupied
                    obstacle_grid[y, x] = 0
        
        # Compute Euclidean distance transform
        self.distance_field = distance_transform_edt(obstacle_grid)

    def heuristic(self, a, b):
        """Euclidean distance heuristic with small tie-breaker"""
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.sqrt(dx**2 + dy**2) * 1.001  # Small tie-breaker
    
    def plan_path(self):
        """Main path planning function"""
        start = (self.start_pose.position.x, self.start_pose.position.y)
        goal = (self.goal_pose.position.x, self.goal_pose.position.y)
        
        # Convert to map coordinates
        start_map = self.world_to_map(*start)
        goal_map = self.world_to_map(*goal)
        
        if not self.is_cell_free(*start_map):
            self.get_logger().error("Start position is occupied!")
            return
        
        if not self.is_cell_free(*goal_map):
            self.get_logger().error("Goal position is occupied!")
            return
        
        # Run Dijkstra's algorithm
        # path_map = self.dijkstra(start_map, goal_map)
        path_map = self.a_star(start_map, goal_map)
        
        if path_map is None:
            return
        
        # Create Path message
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        step = max(1, len(path_map) // self.int_goals)  # Aim for ~20 points

        int_goal_entities = Entities()

        

        arr_x = []
        arr_y = []        
        # Add points to path (with some decimation to reduce number of points)

        for i in range(0, len(path_map), step):
            mx, my = path_map[i]
            x, y = self.map_to_world(mx, my)
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0  # Neutral orientation

            arr_x.append(x)
            arr_y.append(y)
            
            path_msg.poses.append(pose)

        arr_x.append(goal[0])
        arr_y.append(goal[1])


        int_goal_entities.count = len(arr_x)
        int_goal_entities.x = arr_x
        int_goal_entities.y = arr_y

        goal_pos = None
        
        # Ensure goal is included
        if len(path_msg.poses) == 0 or path_msg.poses[-1].pose.position.x != goal[0] or path_msg.poses[-1].pose.position.y != goal[1]:
            mx, my = path_map[-1]
            x, y = self.map_to_world(mx, my)
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)

            goal_pos = pose
        
        self.path_pub.publish(path_msg)
        self.intermediate_goals_pub.publish(int_goal_entities)
        self.publish_global_path(int_goal_entities)



        self.goal_pub.publish(goal_pos)
        
        self.get_logger().info("Published global path!")

    def publish_global_path(self, points):
        marker_array = MarkerArray()

        # ===== 1. LINE STRIP (Connects points with a line) =====
        line_strip_marker = Marker()
        line_strip_marker.header.frame_id = "map"
        line_strip_marker.header.stamp = self.get_clock().now().to_msg()
        line_strip_marker.ns = "global_path"
        line_strip_marker.id = 1000
        line_strip_marker.type = Marker.LINE_STRIP
        line_strip_marker.action = Marker.ADD
        line_strip_marker.scale.x = 0.05  # Line width
        line_strip_marker.color.r = 1.0   # Red
        line_strip_marker.color.b = 1.0   # Blue
        line_strip_marker.color.a = 1.0   # Alpha (opacity)

        # ===== 2. SPHERES (Mark each point individually) =====
        for i in range(len(points.x)):
            # --- Add point to LINE STRIP ---
            line_point = Point()
            line_point.x = float(points.x[i])
            line_point.y = float(points.y[i])
            line_point.z = 0.0
            line_strip_marker.points.append(line_point)

            # --- Create a SPHERE marker for this point ---
            sphere_marker = Marker()
            sphere_marker.header.frame_id = "map"
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = "path_points"
            sphere_marker.id = i  # Unique ID for each point
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position.x = float(points.x[i])
            sphere_marker.pose.position.y = float(points.y[i])
            sphere_marker.pose.position.z = 0.0
            sphere_marker.scale.x = 0.1  # Sphere diameter (X)
            sphere_marker.scale.y = 0.1  # Sphere diameter (Y)
            sphere_marker.scale.z = 0.1  # Sphere diameter (Z)
            sphere_marker.color.g = 1.0  # Green
            sphere_marker.color.a = 1.0  # Fully opaque

            marker_array.markers.append(sphere_marker)

        # Add the line strip to the marker array
        marker_array.markers.append(line_strip_marker)

        # Publish all markers
        self.intermediate_goals_marker.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    planner = GlobalPathPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
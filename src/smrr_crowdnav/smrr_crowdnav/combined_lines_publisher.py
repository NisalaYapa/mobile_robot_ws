import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
from tf2_ros import TransformListener, Buffer
from sklearn.cluster import AgglomerativeClustering
import tf_transformations
from std_msgs.msg import Int16
from smrr_interfaces.msg import Entities

### This node can convert lidar readings to line segments
### This merges lidar reading lines and floor lines (pre-defined)
### Need to create a node to filter out humans from lidar data and send static obs lidar reading to this node.

class StaticObstacles(Node):
    def __init__(self):
        super().__init__('static_obstacle_pulisher')

        # Create a publisher for visualization markers
        self.local_map_subscriber = self.create_subscription(Entities, '/local_lines_array', self.local_callback, 10)
        self.floor_number_subscriber = self.create_subscription(Int16, '/floor_number',self.floor_callback, 10)
        self.marker_publisher = self.create_publisher(Marker, '/combined_line_segments', 10)


        self.local_lines = []
        self.floor_lines = []
        self.combined_lines = []
        self.floor = -1

        # Timer to control marker update rate (e.g., 1 Hz)
        #self.timer = self.create_timer(0.5, self.publish_marker)

    def local_callback(self, msg):
        self.local_lines = msg
        self.local_lines = []
        line_count = msg.count
        for i in range(line_count):
            x1 = msg.x[2*i]
            y1 = msg.y[2*i]
            x2 = msg.x[2*i+1]
            y2 = msg.y[2*i+1]
            line = [[x1, y1],[x2, y2]]
            self.local_lines.append(line)
        self.combine_lines()
        # Extract robot's pose from odometry
        # print(self.local_lines)

    def combine_lines(self):
        self.combined_lines = self.local_lines

        for i in range(len(self.floor_lines)):
            self.combined_lines.append(self.floor_lines[i])
        

        self.publish_marker()

        print(len(self.combined_lines))

    def floor_callback(self, msg):
        self.floor = msg    
        print(self.floor_lines)   
        self.floor_lines = self.extract_known_line()
        print(self.floor_lines)


    def extract_known_line(self):
        # Step 1: Read the file content
        file_path = "/home/nisala/mobile_robot_ws/src/smrr_crowdnav/smrr_crowdnav/maps/my_map.txt"  # Replace with your file 
        with open(file_path, "r") as file:
            print("read")
            lines = file.readlines()

        # Step 2: Parse the file content
        coordinates = []
        for line in lines:
            x, y = map(float, line.strip().split(","))
            coordinates.append([x, y])

        # Step 3: Group coordinates into lines
        # Each pair of rows in the text represents a line
        multi_array = []
        for i in range(0, len(coordinates), 2):
            line_start = coordinates[i]
            line_end = coordinates[i + 1]
            multi_array.append([line_start, line_end])

        # Convert the list to a NumPy array for easier manipulation if needed
        multi_array = np.array(multi_array)

        print(multi_array)
        # Output the result
        return multi_array

    def publish_marker(self):
        # Create a marker for visualization
        marker = Marker()
        marker.header.frame_id = 'map'  # Ensure this matches your frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lines'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Smaller line width for visibility
        marker.color.a = 1.0  # Opacity
        marker.color.r = 1.0  # Red color

        # Add individual line segments to the marker for each cluster
        for cluster in self.combined_lines:
            # Only add lines if there are at least two points in the simplified points
            for i in range(0, len(cluster) - 1):
                start = cluster[i]
                end = cluster[i + 1]

                # Start point of the line segment
                point_start = Point()
                point_start.x = start[0]
                point_start.y = start[1]
                point_start.z = 0.0
                marker.points.append(point_start)

                # End point of the line segment
                point_end = Point()
                point_end.x = end[0]
                point_end.y = end[1]
                point_end.z = 0.0
                marker.points.append(point_end)

        # Publish the marker
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    lidar_line_extraction_node = StaticObstacles()
    rclpy.spin(lidar_line_extraction_node)

    lidar_line_extraction_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

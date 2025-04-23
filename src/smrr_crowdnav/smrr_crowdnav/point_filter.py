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
from smrr_interfaces.msg import Entities
from visualization_msgs.msg import Marker, MarkerArray

class PointExtraction(Node):
    def __init__(self):
        super().__init__('lidar_line_extraction')

        # Parameters
        self.distance_threshold = 0.2  # Threshold for RDP in meters
        self.hac_distance_threshold = 0.5  # Distance threshold for HAC
        self.min_cluster_size = 3  # Minimum points in a cluster

        # TF2 Buffer and Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        #self.marker_publisher = self.create_publisher(Marker, 'line_segments', 10)
        #self.line_publisher = self.create_publisher(Entities, '/local_lines_array', 10)
        self.point_publisher = self.create_publisher(Entities, '/local_points', 10)
        self.point_marker_publisher= self.create_publisher(MarkerArray, '/filterd_points_marker', 10)

        # Subscribers
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer for visualization updates


        # Robot pose and processed marker points
        self.robot_pose = None
        self.merged_lines = []

    def odom_callback(self, msg):
        """Updates robot's pose from odometry data."""
        self.robot_pose = msg.pose.pose


    def polar_to_cartesian(self, angle_min, angle_increment, ranges, max_range=2.0):
        """Convert polar LiDAR scan data to Cartesian coordinates and filter out far points."""
        points = []
        for i, range_val in enumerate(ranges):
            if not np.isinf(range_val) and 0 < range_val <= max_range:  # Ignore infinite, zero, and far points
                angle = angle_min + i * angle_increment
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                points.append((x, y))
        return points


	    
        


    def transform_points(self, points, target_frame="map"):
        """Transforms points from Lidar frame to a target frame (map)."""
        transformed_points = []
        
        point_array = Entities()
        point_array.count = 0
        point_array.x = []
        point_array.y = []
        
            
            
            
        
        for point in points:
            point_stamped = PoseStamped()
            point_stamped.header.frame_id = 'rplidar_link'
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.pose.position.x = point[0]
            point_stamped.pose.position.y = point[1]
            
            
            


            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame, 'rplidar_link', rclpy.time.Time()
                )

                # Extract transformation parameters
                trans = transform.transform.translation
                rot = transform.transform.rotation
                quat = (rot.x, rot.y, rot.z, rot.w)
                roll, pitch, yaw = tf_transformations.euler_from_quaternion(quat)

                # Apply transformation
                x = trans.x + point_stamped.pose.position.x * np.cos(yaw) - point_stamped.pose.position.y * np.sin(yaw)
                y = trans.y + point_stamped.pose.position.x * np.sin(yaw) + point_stamped.pose.position.y * np.cos(yaw)

                transformed_points.append((x, y))
                
                point_array.x.append(x)
                point_array.y.append(y)
                
                
            except Exception as e:
                self.get_logger().warn(f"Transform failed: {e}")
                continue
           
        point_array.count = len(point_array.x)   
        print(f"count {point_array.count}")
        print(f"x {len(point_array.x)}")
	
        self.point_publisher.publish(point_array)
        self.point_marker(point_array)
        return transformed_points




    def scan_callback(self, scan_data):
        """Processes LiDAR scan data to extract and publish line segments."""
        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment
        ranges = scan_data.ranges

        cartesian_points = self.polar_to_cartesian(angle_min, angle_increment, ranges)
        transformed_points = self.transform_points(cartesian_points, "map")

        if not transformed_points:
            self.get_logger().warn("No points after transformation.")
            return


    def point_marker(self, points):
        marker_array = MarkerArray()

        
        # Loop through each human trajectory
        for i in range(points.count):
                # Create a marker for each individual point
                point_marker = Marker()
                point_marker.header.frame_id = "map"
                point_marker.header.stamp = self.get_clock().now().to_msg()
                point_marker.ns = f"point"
                point_marker.id = i# Unique ID for each point
                point_marker.type = Marker.SPHERE
                point_marker.action = Marker.ADD
                point_marker.scale.x = 0.1  # Adjust scale for visibility
                point_marker.scale.y = 0.1
                point_marker.scale.z = 0.0
                point_marker.color.r = 0.0  # Red color for visibility
                point_marker.color.g = 1.0
                point_marker.color.b = 0.0
                point_marker.color.a = 1.0  # Fully opaque
                point_marker.lifetime = rclpy.time.Duration(seconds=0.5).to_msg()  # Markers persist for 5 seconds

                # Set the position of the marker
                point_marker.pose.position.x = float(points.x[i])
                point_marker.pose.position.y = float(points.y[i])
                point_marker.pose.position.z = 0.0

                # Add each point as a separate marker in the MarkerArray
                marker_array.markers.append(point_marker)

        # Publish all points as separate markers
        #print(marker_array)
        self.point_marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PointExtraction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
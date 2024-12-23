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

### This node can convert lidar readings to line segments
### Need to create a node to filter out humans from lidar data and send static obs lidar reading to this node.

class LidarLineExtraction(Node):
    def __init__(self):
        super().__init__('lidar_line_extraction')

        # Parameters
        self.distance_threshold = 0.5  # Threshold for RDP in meters
        self.hac_distance_threshold = 0.8  # Distance threshold for HAC
        self.min_cluster_size = 5  # Minimum points in a cluster

        # TF2 Buffer and Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a publisher for visualization markers
        #self.marker_publisher = self.create_publisher(Marker, 'line_segments', 10)
        self.line_publisher = self.create_publisher(Entities, '/local_lines_array', 10)

        # Subscribe to the /scan topic
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Subscribe to the /odom topic to get robot's pose
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Timer to control marker update rate (e.g., 1 Hz)
        #self.timer = self.create_timer(1.0, self.publish_marker)

        # Placeholder for robot pose and processed marker points
        self.robot_pose = None
        self.merged_lines = []

    def odom_callback(self, msg):
        # Extract robot's pose from odometry
        self.robot_pose = msg.pose.pose

    def polar_to_cartesian(self, angle_min, angle_increment, ranges):
        points = []
        for i, range_val in enumerate(ranges):
            if not np.isinf(range_val):  # Avoiding infinite ranges
                angle = angle_min + i * angle_increment
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                points.append((x, y))
        return points

    def transform_points(self, points, target_frame):
        # Transform points to the map frame
        transformed_points = []
        for point in points:
            point_stamped = PoseStamped()
            point_stamped.header.frame_id = 'base_scan'
            point_stamped.pose.position.x = point[0]
            point_stamped.pose.position.y = point[1]

            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    point_stamped.header.frame_id,
                    rclpy.time.Time()
                )

                # Extract quaternion from the transform
                quat = (
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                )

                # Convert quaternion to Euler angles
                roll, pitch, yaw = tf_transformations.euler_from_quaternion(quat)

                # Transform point to target frame using yaw
                x = (transform.transform.translation.x + 
                     point_stamped.pose.position.x * np.cos(yaw) - 
                     point_stamped.pose.position.y * np.sin(yaw))
                y = (transform.transform.translation.y + 
                     point_stamped.pose.position.x * np.sin(yaw) + 
                     point_stamped.pose.position.y * np.cos(yaw))

                transformed_points.append((x, y))

            except Exception as e:
                self.get_logger().warn(f"Transform failed: {e}")
                continue

        return transformed_points

    def cluster_points(self, points):
        if len(points) == 0:
            self.get_logger().warn("No points received for clustering.")
            return []

        points = np.array(points)
        if points.ndim != 2 or points.shape[1] != 2:
            self.get_logger().warn("Clustering requires a 2D array with x and y coordinates.")
            return []

        # Hierarchical Agglomerative Clustering (HAC) parameters
        hac = AgglomerativeClustering(n_clusters=None, distance_threshold=self.hac_distance_threshold)

        # Fit HAC to the data and obtain labels
        labels = hac.fit_predict(points)

        # Collect clusters
        clusters = []
        for cluster_id in set(labels):
            cluster_points = points[labels == cluster_id]
            if len(cluster_points) >= self.min_cluster_size:
                clusters.append(cluster_points.tolist())

        return clusters

    def rdp(self, points, epsilon):
        if len(points) < 3:
            return points

        start, end = np.array(points[0]), np.array(points[-1])
        dmax = 0
        index = -1
        for i in range(1, len(points) - 1):
            d = np.abs(np.cross(end - start, start - np.array(points[i]))) / np.linalg.norm(end - start)
            if d > dmax:
                index = i
                dmax = d

        if dmax > epsilon:
            left = self.rdp(points[:index + 1], epsilon)
            right = self.rdp(points[index:], epsilon)
            return left[:-1] + right
        else:
            return [start.tolist(), end.tolist()]

    def merge_clusters(self, clusters):
        merged_lines = []

        line_array = Entities()

        line_array.count       = 0
        line_array.x           = []
        line_array.y           = []
        
        for cluster in clusters:
            if self.is_valid_cluster(cluster):
                simplified_points = self.rdp(cluster, self.distance_threshold)
                if len(simplified_points) >= 2:
                    merged_lines.append(simplified_points)  # Append the entire simplified line segment
                    line_array.x.append(simplified_points[0][0])
                    line_array.x.append(simplified_points[1][0])
                    line_array.y.append(simplified_points[0][1])
                    line_array.y.append(simplified_points[1][1])
                    line_array.count +=  1
        print(line_array.count)
        self.line_publisher.publish(line_array)

        return merged_lines

    def is_valid_cluster(self, cluster):
        # Check if the maximum distance between points in the cluster is within a valid range
        cluster_points = np.array(cluster)
        dists = np.linalg.norm(cluster_points[:, np.newaxis] - cluster_points, axis=2)
        if np.max(dists) > self.distance_threshold * 3:  # Adjust the multiplier as needed
            return False
        return True

    def filter_lines(self, lines):
        filtered_lines = []
        for i in range(0, len(lines) - 1, 2):
            if i + 1 < len(lines):
                start = np.array(lines[i])
                end = np.array(lines[i + 1])
                length = np.linalg.norm(end - start)
                # Only keep lines longer than a certain threshold (e.g., 0.1m)
                if length > 0.1:
                    filtered_lines.append(start.tolist())
                    filtered_lines.append(end.tolist())
        return filtered_lines

    def scan_callback(self, scan_data):
        # Extract LiDAR parameters and data
        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment
        ranges = scan_data.ranges

        # Convert to Cartesian coordinates
        cartesian_points = self.polar_to_cartesian(angle_min, angle_increment, ranges)

        # Transform points to map frame
        cartesian_points = self.transform_points(cartesian_points, "map")

        # Ensure cartesian_points has data before proceeding
        if not cartesian_points:
            self.get_logger().warn("No points to process after transformation.")
            return

        # Cluster points
        clusters = self.cluster_points(cartesian_points)

        # Merge clusters and estimate line segments
        self.merged_lines = self.merge_clusters(clusters)

    # def publish_marker(self):
    #     # Create a marker for visualization
    #     marker = Marker()
    #     marker.header.frame_id = 'map'  # Ensure this matches your frame
    #     marker.header.stamp = self.get_clock().now().to_msg()
    #     marker.ns = 'lines'
    #     marker.id = 0
    #     marker.type = Marker.LINE_LIST
    #     marker.action = Marker.ADD
    #     marker.scale.x = 0.05  # Smaller line width for visibility
    #     marker.color.a = 1.0  # Opacity
    #     marker.color.r = 1.0  # Red color

    #     # Add individual line segments to the marker for each cluster
    #     for cluster in self.merged_lines:
    #         # Only add lines if there are at least two points in the simplified points
    #         for i in range(0, len(cluster) - 1):
    #             start = cluster[i]
    #             end = cluster[i + 1]

    #             # Start point of the line segment
    #             point_start = Point()
    #             point_start.x = start[0]
    #             point_start.y = start[1]
    #             point_start.z = 0.0
    #             marker.points.append(point_start)

    #             # End point of the line segment
    #             point_end = Point()
    #             point_end.x = end[0]
    #             point_end.y = end[1]
    #             point_end.z = 0.0
    #             marker.points.append(point_end)

    #     # Publish the marker
    #     self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    lidar_line_extraction_node = LidarLineExtraction()
    rclpy.spin(lidar_line_extraction_node)

    lidar_line_extraction_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

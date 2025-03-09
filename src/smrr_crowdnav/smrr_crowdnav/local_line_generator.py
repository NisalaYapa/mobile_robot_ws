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

class LidarLineExtraction(Node):
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
        self.marker_publisher = self.create_publisher(Marker, 'line_segments', 10)
        self.line_publisher = self.create_publisher(Entities, '/local_lines_array', 10)

        # Subscribers
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan_fake', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer for visualization updates
        self.timer = self.create_timer(1.0, self.publish_marker)

        # Robot pose and processed marker points
        self.robot_pose = None
        self.merged_lines = []

    def odom_callback(self, msg):
        """Updates robot's pose from odometry data."""
        self.robot_pose = msg.pose.pose

    def polar_to_cartesian(self, angle_min, angle_increment, ranges):
        """Convert polar LiDAR scan data to Cartesian coordinates."""
        points = []
        for i, range_val in enumerate(ranges):
            if not np.isinf(range_val) and range_val > 0:  # Avoid infinite or invalid ranges
                angle = angle_min + i * angle_increment
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                points.append((x, y))
        return points

    def transform_points(self, points, target_frame="map"):
        """Transforms points from Lidar frame to a target frame (map)."""
        transformed_points = []
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
            except Exception as e:
                self.get_logger().warn(f"Transform failed: {e}")
                continue

        return transformed_points

    def cluster_points(self, points):
        """Cluster points using Hierarchical Agglomerative Clustering (HAC)."""
        if len(points) == 0:
            self.get_logger().warn("No points received for clustering.")
            return []

        points = np.array(points)
        if points.ndim != 2 or points.shape[1] != 2:
            self.get_logger().warn("Invalid point format for clustering.")
            return []

        # Apply HAC
        hac = AgglomerativeClustering(n_clusters=None, distance_threshold=self.hac_distance_threshold)
        labels = hac.fit_predict(points)

        # Collect clusters
        clusters = []
        for cluster_id in set(labels):
            cluster_points = points[labels == cluster_id]
            if len(cluster_points) >= self.min_cluster_size:
                clusters.append(cluster_points.tolist())

        return clusters

    def rdp(self, points, epsilon):
        """Ramer-Douglas-Peucker line simplification algorithm."""
        if len(points) < 2:
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
        """Merges clusters into line segments and publishes to the topic."""
        merged_lines = []
        line_array = Entities()
        line_array.count = 0
        line_array.x = []
        line_array.y = []

        for cluster in clusters:
            if len(cluster) >= 2:
                simplified_points = self.rdp(cluster, self.distance_threshold)
                if len(simplified_points) >= 2:
                    merged_lines.append(simplified_points)
                    line_array.x.append(simplified_points[0][0])
                    line_array.x.append(simplified_points[1][0])
                    line_array.y.append(simplified_points[0][1])
                    line_array.y.append(simplified_points[1][1])
                    line_array.count += 1

        self.line_publisher.publish(line_array)
        return merged_lines

    def publish_marker(self):
        """Publishes visualization markers."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lines"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for line in self.merged_lines:
            for point in line:
                p = Point()
                p.x, p.y, p.z = point[0], point[1], 0.0
                marker.points.append(p)

        self.marker_publisher.publish(marker)

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

        clusters = self.cluster_points(transformed_points)
        self.merged_lines = self.merge_clusters(clusters)

def main(args=None):
    rclpy.init(args=args)
    node = LidarLineExtraction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
import math

# Parameters
douglas_peucker_distance = 0.1
neighbor_distance = 0.5
min_cluster_size = 5

class LineSegmentExtractor(Node):
    def __init__(self):
        super().__init__('extract_line_segments_node')
        self.marker_pub = self.create_publisher(Marker, 'line_segments', 2)
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_cb, 2)
        self.lp = LaserProjection()
        
        # Parameters from ROS parameters server or defaults
        self.declare_parameter("douglas_pecker_distance", 0.1)
        self.declare_parameter("neighbor_distance", 0.5)
        self.declare_parameter("min_cluster_size", 5)
        self.douglas_pecker_distance = self.get_parameter("douglas_pecker_distance").value
        self.neighbor_distance = self.get_parameter("neighbor_distance").value
        self.min_cluster_size = self.get_parameter("min_cluster_size").value

    def laser_cb(self, msg):
        # Create marker for RViz visualization
        line_segments = Marker()
        line_segments.header.frame_id = msg.header.frame_id
        line_segments.header.stamp = self.get_clock().now().to_msg()
        line_segments.ns = "line_segments"
        line_segments.action = Marker.ADD
        line_segments.type = Marker.LINE_LIST
        line_segments.pose.orientation.w = 1.0
        line_segments.id = 42
        line_segments.scale.x = 0.1
        line_segments.color.r = 1.0
        line_segments.color.a = 1.0

        # Convert LaserScan data to PointCloud2 using LaserProjection
        cloud = self.lp.projectLaser(msg)
        
        # Initialize the cluster
        cluster = []
        
        # Process each point in PointCloud2 data
        for point in pc2.read_points(cloud, field_names=("x", "y"), skip_nans=True):
            x, y = point[:2]
            current_point = Point()
            current_point.x = x
            current_point.y = y
            cluster.append(current_point)

            is_cluster_ready = False

            # Check if we need to start a new cluster based on distance between points
            if len(cluster) > 1:
                prev_point = cluster[-2]
                distance = math.sqrt((current_point.x - prev_point.x)**2 + (current_point.y - prev_point.y)**2)
                is_cluster_ready = distance > self.neighbor_distance

            # Process the cluster if it's ready
            if is_cluster_ready or point == list(pc2.read_points(cloud, skip_nans=True))[-1]:
                if len(cluster) >= self.min_cluster_size:
                    # Simplify cluster with Douglas-Peucker algorithm
                    simplified = self.simplify_cluster(cluster)

                    # Add simplified points to Marker for visualization
                    for i in range(len(simplified) - 1):
                        line_segments.points.append(simplified[i])
                        line_segments.points.append(simplified[i + 1])

                # Clear the cluster to start a new one
                cluster.clear()

        # Publish the line segments for visualization
        self.marker_pub.publish(line_segments)

    def simplify_cluster(self, cluster):
        """Simplify the cluster using the Douglas-Peucker algorithm."""
        simplified = [cluster[0]]
        for i in range(1, len(cluster) - 1):
            prev_point = simplified[-1]
            next_point = cluster[i + 1]
            distance = math.sqrt((next_point.x - prev_point.x)**2 + (next_point.y - prev_point.y)**2)
            if distance > self.douglas_pecker_distance:
                simplified.append(cluster[i])
        simplified.append(cluster[-1])
        return simplified


def main(args=None):
    rclpy.init(args=args)
    node = LineSegmentExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


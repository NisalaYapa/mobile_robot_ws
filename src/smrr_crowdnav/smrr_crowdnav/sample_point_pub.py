import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from smrr_interfaces.msg import Entities
import random

class FakeLidarPublisher(Node):
    def __init__(self):
        super().__init__('fake_lidar_publisher')

        # Publishers
        self.marker_publisher = self.create_publisher(Marker, 'lidar_markers', 10)
        self.point_publisher = self.create_publisher(Entities, '/local_points', 10)

        # Timer for periodic updates
        self.timer = self.create_timer(1.0, self.publish_lidar_points)

        # Generate lidar-like obstacle points
        self.lidar_points = self.generate_lidar_obstacles()

    def generate_lidar_obstacles(self):
        """Generate points simulating a hallway with two parallel walls."""
        points = []

        hallway_length = 10  # Length along x-axis
        wall_y1, wall_y2 = 3, -3  # Positions of the walls
        num_points = 50  # Density of lidar points along the wall
        doorway_x = 4  # Position of the doorway gap
        doorway_width = 2  # Width of the doorway opening

        # Left wall (continuous)
        left_wall = [(x, wall_y1) for x in self.linspace(-hallway_length / 2, hallway_length / 2, num_points)]

        # Right wall (with a doorway gap)
        right_wall = [(x, wall_y2) for x in self.linspace(-hallway_length / 2, doorway_x - doorway_width / 2, num_points // 2)]
        right_wall += [(x, wall_y2) for x in self.linspace(doorway_x + doorway_width / 2, hallway_length / 2, num_points // 2)]

        # Add walls to points
        points.extend(left_wall)
        points.extend(right_wall)

        # Add noise to simulate real-world lidar inaccuracies
        noisy_points = [(x + random.uniform(-0.05, 0.05), y + random.uniform(-0.05, 0.05)) for x, y in points]

        return noisy_points


    def linspace(self, start, end, num):
        """Generate evenly spaced points between start and end."""
        return [start + i * (end - start) / (num - 1) for i in range(num)]

    def publish_lidar_points(self):
        """Publishes points simulating lidar scans of objects."""
        lidar_data = Entities()
        lidar_data.count = len(self.lidar_points)

        # Fill point data
        for x, y in self.lidar_points:
            lidar_data.x.append(float(x))
            lidar_data.y.append(float(y))

        # Publish the lidar points
        self.point_publisher.publish(lidar_data)

        # Publish the visualization marker for RViz
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lidar_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Smaller dots for lidar-like points
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0  # Red points for lidar readings
        marker.color.g = 0.0
        marker.color.b = 0.0

        for x, y in self.lidar_points:
            p = Point(x=float(x), y=float(y), z=0.0)
            marker.points.append(p)

        # Publish the visualization marker
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

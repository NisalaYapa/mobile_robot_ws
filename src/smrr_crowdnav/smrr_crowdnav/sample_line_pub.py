import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from smrr_interfaces.msg import Entities
import math

class FakeLinePublisher(Node):
    def __init__(self):
        super().__init__('fake_line_publisher')

        # Publishers
        self.marker_publisher = self.create_publisher(Marker, 'line_segments', 10)
        self.line_publisher = self.create_publisher(Entities, '/local_lines_array', 10)

        # Timer for visualization updates
        self.timer = self.create_timer(1.0, self.publish_fake_lines)

        # Variables for fake line generation
        self.circle_lines = self.generate_circle_lines()

    def generate_circle_lines(self):
        """Generate lines to approximate a circle with radius 10 using 50 short line segments."""
        radius = 10.0
        num_lines = 50  # Updated to 50 line segments
        angle_step = 2 * math.pi / num_lines  # Divide the circle into equal angles
        circle_lines = []

        for i in range(num_lines):
            # Compute the start and end points of each line segment
            angle_start = i * angle_step
            angle_end = (i + 1) * angle_step

            x_start = radius * math.cos(angle_start)
            y_start = radius * math.sin(angle_start)
            x_end = radius * math.cos(angle_end)
            y_end = radius * math.sin(angle_end)

            # Create the line segment
            circle_lines.append([(x_start, y_start), (x_end, y_end)])

        return circle_lines

    def publish_fake_lines(self):
        """Publishes circle lines for visualization."""
        circle_line_array = Entities()
        circle_line_array.count = len(self.circle_lines)

        # Fill line segment data
        for line in self.circle_lines:
            # Define start and end points for each line
            x_start, y_start = line[0]
            x_end, y_end = line[1]

            # Ensure the values are float
            start_point = Point(x=float(x_start), y=float(y_start), z=0.0)
            end_point = Point(x=float(x_end), y=float(y_end), z=0.0)

            # Add points to the Entities message
            circle_line_array.x.append(start_point.x)
            circle_line_array.y.append(start_point.y)
            circle_line_array.x.append(end_point.x)
            circle_line_array.y.append(end_point.y)

        # Publish the circle line segments
        self.line_publisher.publish(circle_line_array)

        # Publish the marker for visualization in RViz
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "circle_lines"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0  # Green color for circle lines
        marker.color.b = 0.0

        for line in self.circle_lines:
            for point in line:
                p = Point(x=float(point[0]), y=float(point[1]), z=0.0)  # Ensure each point is a float
                marker.points.append(p)

        # Publish the visualization marker
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = FakeLinePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

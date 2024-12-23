import rclpy
from rclpy.node import Node
from .include.marker_visualizer import MarkerVisualizer

class Visualizer(Node):
    def __init__(self):
        super().__init__("visualizer")

        self.declare_parameter("subscriber_topic", "/laser_data_array")
        self.declare_parameter("publisher_topic", "/lidar_markers")
        self.declare_parameter("marker_frame", "map")
        self.declare_parameter("marker_color", "red")

        self.subscriber_topic = self.get_parameter("subscriber_topic").get_parameter_value().string_value
        self.publisher_topic = self.get_parameter("publisher_topic").get_parameter_value().string_value
        self.marker_frame = self.get_parameter("marker_frame").get_parameter_value().string_value
        self.marker_color = self.get_parameter("marker_color").get_parameter_value().string_value

        self.get_logger().info("Subscriber topic is set to %s" % self.subscriber_topic)
        self.get_logger().info("Publisher topic is set to %s" % self.publisher_topic)
        self.get_logger().info("Marker frame is set to %s" % self.marker_frame)
        self.get_logger().info("Marker color is set to %s" % self.marker_color)

        self.marker_visualizer = MarkerVisualizer(self, 
                                                  self.subscriber_topic,
                                                  self.publisher_topic,
                                                  self.marker_frame,
                                                  self.marker_color
                                                  )

def main(args=None):
    rclpy.init(args=args)
    visualizer = Visualizer()
    rclpy.spin(visualizer)
    rclpy.shutdown()
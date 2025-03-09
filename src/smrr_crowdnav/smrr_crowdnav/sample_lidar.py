import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class FakeLidarPublisher(Node):
    def __init__(self):
        super().__init__('fake_lidar_publisher')
        self.publisher = self.create_publisher(LaserScan, '/scan_fake', 10)
        self.timer = self.create_timer(0.1, self.publish_lidar_data)  # 10Hz

        self.angle_min = -np.pi / 3  # -60 degrees
        self.angle_max = np.pi / 3   # 60 degrees
        self.angle_increment = np.pi / 180  # 1-degree increments
        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        self.max_range = 5.0  # Max LiDAR range in meters
        self.obstacle_positions = [(-1, 2), (1, 3), (0, 1.5)]  # (x, y) coordinates of obstacles

    def generate_lidar_readings(self):
        """ Simulates LiDAR readings with a 120° field of view. """
        ranges = np.full(self.num_readings, self.max_range)  # Default: Max range

        for obj_x, obj_y in self.obstacle_positions:
            angle = np.arctan2(obj_y, obj_x)  # Compute angle of object
            if self.angle_min <= angle <= self.angle_max:
                index = int((angle - self.angle_min) / self.angle_increment)
                distance = np.sqrt(obj_x**2 + obj_y**2)
                ranges[index] = min(distance, self.max_range)  # Simulated obstacle distance

        noise = np.random.normal(0, 0.05, self.num_readings)  # Small noise
        ranges = np.clip(ranges + noise, 0.1, self.max_range)  # Ensure valid ranges
        return ranges.tolist()

    def publish_lidar_data(self):
        """ Publishes a realistic fake LiDAR scan with 120° FoV """
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "rplidar_link"
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = self.max_range
        scan.ranges = self.generate_lidar_readings()
        self.publisher.publish(scan)
        self.get_logger().info("Published 120° Fake LiDAR Scan")

def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

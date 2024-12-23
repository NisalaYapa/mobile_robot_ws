import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from smrr_interfaces.msg import Entities
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from visualization_msgs.msg import Marker, MarkerArray
from .include.transform import GeometricTransformations

# Global variables
visual_data   = Entities()
lidar_data    = []

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")
        self.cb_group      = ReentrantCallbackGroup()
        self.laser_sub_    = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10, callback_group=self.cb_group)
        self.laser_pub_    = self.create_publisher(Entities, "/laser_data_array", 10)
        self.transform     = GeometricTransformations(self)

        self.get_logger().info("Lidar subscriber node has been started")

    def lidar_callback(self, lidar_data_msg):
        global lidar_data, visual_data

        lidar_ranges          = np.array(lidar_data_msg.ranges)
        lidar_angle_min       = lidar_data_msg.angle_min
        lidar_angle_increment = lidar_data_msg.angle_increment

        lidar_angles          = np.arange(lidar_angle_min, lidar_angle_min + len(lidar_ranges) * lidar_angle_increment, lidar_angle_increment)
        lidar_x               = lidar_ranges * np.cos(lidar_angles)
        lidar_y               = lidar_ranges * np.sin(lidar_angles)

        lidar_points = np.vstack((lidar_x, lidar_y)).T

        # variables for tracking the person
        lidar_th              = 0.75
        threshold             = lidar_th
        classes               = []
        updated_lidar_data    = []

        for i, data in enumerate(lidar_data):
            person_x, person_y, class_   = data[:]
            distances                    = np.linalg.norm(lidar_points - np.array([person_x, person_y], dtype=float), axis=1)
            points_within_circle         = np.where(distances <= threshold)[0]

            if len(points_within_circle) == 0:
                self.get_logger().warn(f"Person {i} not found in LiDAR data")
                continue

            mean_lidar_x   = np.mean(lidar_points[points_within_circle, 0])
            mean_lidar_y   = np.mean(lidar_points[points_within_circle, 1])

            updated_lidar_data.append([mean_lidar_x, mean_lidar_y, class_])
        
        lidar_data   = updated_lidar_data

        visual_points_classes   = np.array(visual_data.classes)
        visual_points_x         = np.array(visual_data.x, dtype=float)
        visual_points_y         = np.array(visual_data.y, dtype=float)
        visual_points           = np.vstack((visual_points_x, visual_points_y, visual_points_classes)).T

        if len(lidar_data) == 0:
            lidar_data     = visual_points.tolist()
        else:
            threshold = 2.0
            new_lidar_data = lidar_data.copy()

            for i, visual_point in enumerate(visual_points):
                lidar_data_np    = np.array(lidar_data)[:, :2]
                lidar_data_np    = np.array(lidar_data_np, dtype=float)
                distances        = np.linalg.norm(lidar_data_np - np.array(visual_point[:2], dtype=float), axis=1)
                min_distance     = np.min(distances)

                if min_distance > threshold:
                    new_lidar_data.append(visual_point.tolist())

            unique_lidar_data = []

            for i, point in enumerate(new_lidar_data):
                if point not in unique_lidar_data:
                    unique_lidar_data.append(point)

            lidar_data      = unique_lidar_data
            lidar_data      = np.array(lidar_data)
            lidar_data_xy   = lidar_data[:, :2]
            lidar_data_xy   = np.array(lidar_data_xy, dtype=float)

            # transform the lidar data to the map frame
            transformation = self.transform.get_transform("map", "rplidar_link")
            obs_arr        = np.hstack((lidar_data_xy[:,0], lidar_data_xy[:,1], np.zeros_like(lidar_data_xy[:,0]), np.ones_like(lidar_data_xy[:,0]))).reshape(4,-1)

            if transformation is not None:
                transformed_points      = self.transform.transform_points(obs_arr, transformation)
                transformed_lidar_data  = transformed_points.T

            transformed_lidar_data  = np.array(transformed_lidar_data)
            
            entities          = Entities()
            entities.count    = len(lidar_data)

            arr_classes       = lidar_data[:, 2]
            arr_classes       = np.array(arr_classes, dtype=str)
            entities.classes  = arr_classes.tolist()

            arr_x             = transformed_lidar_data[:, 0]
            arr_x             = np.array(arr_x, dtype=float)
            entities.x        = arr_x.tolist()

            arr_y             = transformed_lidar_data[:, 1]
            arr_y             = np.array(arr_y, dtype=float)
            entities.y        = arr_y.tolist()

            self.laser_pub_.publish(entities)
  

class VisualDataSubscriber(Node):
    def __init__(self):
        super().__init__("visual_data_subscriber")
        self.cb_group    = ReentrantCallbackGroup()
        self.visual_sub_ = self.create_subscription(Entities, "/visual_dynamic_obs_array", self.visual_callback, 10, callback_group=self.cb_group)

    def visual_callback(self, visual_msg):
        global visual_data, lidar_data
        visual_data = visual_msg

        
def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber       = LidarSubscriber()
    visual_data_subscriber = VisualDataSubscriber()

    executor = MultiThreadedExecutor()
    executor.add_node(lidar_subscriber)
    executor.add_node(visual_data_subscriber)
    executor.spin()

if __name__ == "__main__":
    main()


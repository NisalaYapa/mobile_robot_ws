from ultralytics import YOLO
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
from smrr_interfaces.msg import Entities, Footprint
import math
import torch
from .include.inference_node_utils import InferenceNodeUtils

bridge = CvBridge()

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        self.model   = YOLO('/home/nisala/mobile_robot_ws/src/object_tracker/object_tracker/best_nano.pt')

        # self.sub_       = self.create_subscription(Image, '/color/image_raw', self.camera_callback, 10)
        self.img_pub_   = self.create_publisher(Image, '/object_tracker/inference_result', 1)
        self.array_pub_ = self.create_publisher(Entities, '/object_tracker/visual_dynamic_obs_array', 1)
        self.footprint_pub_ = self.create_publisher(Footprint, '/object_tracker/footprint_array', 1)

        self.sub_color = self.create_subscription(Image, '/color/image_raw', self.camera_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.sub_camera_info = self.create_subscription(CameraInfo, '/aligned_depth_to_color/camera_info', self.camera_info_callback, 10)

        self.camera_matrix = []
        self.is_camera_matrix = False
        self.is_depth_image = False

        self.inference_node_utils = InferenceNodeUtils(model=self.model)
        
        self.get_logger().info("Inference node has been started.")


    def camera_info_callback(self, msg):
        self.is_camera_matrix = True
        self.camera_matrix = np.array(msg.k).reshape(3, 3)  # Extracting 3x3 intrinsic matrix

    def depth_callback(self, msg):
        self.is_depth_image = True
        self.depth_image = bridge.imgmsg_to_cv2(msg, "16UC1")  # Depth in millimeters        
        

    def camera_callback(self, msg):
        img         = bridge.imgmsg_to_cv2(msg, "bgr8")
        results     = self.model(img)

        try:
            xyxy_boxes = results[0].boxes.xyxy  # Bounding box coordinates (x1, y1, x2, y2)
            class_indices = results[0].boxes.cls  # Class indices
            class_velocities = torch.tensor([1.5, 0.8, 1.0, 1.2])
            velocities = class_velocities[class_indices.long()]
            new_data = torch.cat((xyxy_boxes, class_indices.unsqueeze(1), velocities.unsqueeze(1)), dim=1)

        except AttributeError:
            self.get_logger().warn("No objects detected or invalid attributes in results.")
        except Exception as e:
            pass

        mid_point_x = int(img.shape[1] / 2)
        if ((self.is_camera_matrix) and (self.is_depth_image)) :
            classes, arr_x, arr_y , width = self.inference_node_utils.coordinates_from_camera(results, img, mid_point_x, self.camera_matrix, self.depth_image)

            entities = Entities()
            entities.count = len(arr_x)
            entities.classes = classes
            entities.x = arr_x
            entities.y = arr_y

            self.array_pub_.publish(entities)

            footprint = Footprint()
            footprint.count = len(width)
            footprint.d = width

            self.footprint_pub_.publish(footprint)


            img_msg = bridge.cv2_to_imgmsg(img)
            # self.img_pub_.publish(img_msg)
    
       
def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)

    camera_subscriber.destroy_node()
    rclpy.shutdown()


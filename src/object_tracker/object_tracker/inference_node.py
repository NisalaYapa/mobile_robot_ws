from ultralytics import YOLO
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from smrr_interfaces.msg import Entities
from visualization_msgs.msg import Marker, MarkerArray
import math
import torch

bridge = CvBridge()

def distance_from_camera(height):
    return (310 * 3) / height

def angle_from_camera(hor_dis):
    return (hor_dis * 45.07) / 557

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        self.model   = YOLO('/home/nisala/mobile_robot_ws/src/object_tracker/object_tracker/best.pt')

        self.sub_       = self.create_subscription(Image, '/zed2_left_camera/image_raw', self.camera_callback, 10)
        self.img_pub_   = self.create_publisher(Image, '/inference_result', 1)
        self.array_pub_ = self.create_publisher(Entities, '/visual_dynamic_obs_array', 1)

        self.lidar_to_cam_opt = np.array([
            [ 0.000  , 0.000 , 1.000 ,-0.170  ],
            [ -1.000 , 0.000 , 0.000 , 0.060  ],
            [ 0.000  ,-1.000 , 0.000 , 1.099  ],
            [ 0.000  , 0.000 , 0.000 , 1.000  ]
        ])

    def camera_callback(self, msg):
        img         = bridge.imgmsg_to_cv2(msg, "bgr8")
        results     = self.model(img)

        try:
            # Assuming results[0].boxes.xyxy and results[0].boxes.cls are already tensors
            xyxy_boxes = results[0].boxes.xyxy  # Bounding box coordinates (x1, y1, x2, y2)
            class_indices = results[0].boxes.cls  # Class indices

            # Define the float values for each class
            class_velocities = torch.tensor([1.5, 0.8, 1.0, 1.2])

            # Use class_indices to index the class_floats tensor and get the corresponding float values
            velocities = class_velocities[class_indices.long()]

            # Concatenate the bounding boxes, class indices, and float values into a single tensor
            new_data = torch.cat((xyxy_boxes, class_indices.unsqueeze(1), velocities.unsqueeze(1)), dim=1)

            # Print the final tensor
            print(new_data)

            print("\nPreferred velocity tensor:")
            print(results[0].names)
            print()
            results[0].velocity_tensor = new_data
            print(results[0].velocity_tensor)

        except AttributeError:
            self.get_logger().warn("No objects detected or invalid attributes in results.")
        except Exception as e:
            pass

        mid_point_x = int(img.shape[1] / 2)
        mid_point_y = int(img.shape[0] / 2)

        classes     = []
        arr_x       = []
        arr_y       = []

        for r in (results):
            boxes = r.boxes
            
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                conf = box.conf.item()
                if (self.model.names[int(c)] == "normal-adult"):

                    x_min = int(b[0])  # Top-left x-coordinate
                    y_min = int(b[1])  # Top-left y-coordinate
                    x_max = int(b[2])  # Bottom-right x-coordinate
                    y_max = int(b[3])  # Bottom-right y-coordinate

                    # Draw rectangle
                    cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (255, 255, 0), thickness=2)

                    # Height calculations
                    height   = y_max - y_min
                    distance = distance_from_camera(height)

                    # Angle calculations, distance corrections
                    u                  = int((x_min + x_max) / 2)  # Center x-coordinate of BB 
                    horizontal_pixel   = u - mid_point_x
                    angle              = angle_from_camera(horizontal_pixel)
                    distance           = distance / np.cos(np.radians(angle))

                    cam_x = distance * math.sin(math.radians(angle))
                    cam_y = 0.0
                    cam_z = distance * math.cos(math.radians(angle))

                    # multiply by the transformation matrix
                    lidar_x, lidar_y, lidar_z, _ = np.dot(self.lidar_to_cam_opt, [cam_x, cam_y, cam_z, 1])

                    # Draw label
                    label = f'D: {distance:.2f}m A: {angle:.2f} deg'
                    cv2.putText(img, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)

                    classes.append(self.model.names[int(c)])
                    arr_x.append(lidar_x)
                    arr_y.append(lidar_y)

        entities = Entities()
        entities.count = len(arr_x)
        entities.classes = classes
        entities.x = arr_x
        entities.y = arr_y

        self.array_pub_.publish(entities)
        img_msg = bridge.cv2_to_imgmsg(img)
        self.img_pub_.publish(img_msg)
        
       
def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)

    camera_subscriber.destroy_node()
    rclpy.shutdown()


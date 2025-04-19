import numpy as np
import cv2
import math

class InferenceNodeUtils():
    def __init__(self, model):
        self.model = model

        self.lidar_to_cam_opt = np.array([
            [ 0.000  , 0.000 , 1.000 ,-0.170  ],
            [ -1.000 , 0.000 , 0.000 , 0.060  ],
            [ 0.000  ,-1.000 , 0.000 , 1.099  ],
            [ 0.000  , 0.000 , 0.000 , 1.000  ]
        ])

    def distance_from_camera(self, height):
        return (310 * 3) / height
    
    def angle_from_camera(self, hor_dis):
        return (hor_dis * 45.07) / 557
    
    def coordinates_from_camera(self, results, img, mid_point_x, camera_matrix, depth_image):
        classes, arr_x, arr_y, arr_width = [], [], [], []

        fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]  # Focal lengths
        cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]  # Principal point

        for r in (results):
            boxes = r.boxes
            
            for box in boxes:
                b    = box.xyxy[0].to('cpu').detach().numpy().copy()
                c    = box.cls
                conf = box.conf.item()
                if (self.model.names[int(c)] == "normal-adult"):

                    x_min = int(b[0])  # Top-left x-coordinate
                    y_min = int(b[1])  # Top-left y-coordinate
                    x_max = int(b[2])  # Bottom-right x-coordinate
                    y_max = int(b[3])  # Bottom-right y-coordinate

                    cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (255, 255, 0), thickness=2)                          # Draw rectangle

                    # Get center of the bounding box
                    cx_bb = (x_min + x_max) // 2
                    cy_bb = (y_min + y_max) // 2

                    # Get depth at center of bounding box
                    cam_z = depth_image[cy_bb, cx_bb] / 1000.0  # Convert mm to meters

                    if cam_z < 0: 
                        continue # Ignore invalid depth readings

                    # Compute real-world 3D coordinates (X, Y, Z) w.r.t optical frame of camera
                    cam_x = (cx_bb - cx) * cam_z / fx
                    # cam_y = (cy_bb - cy) * cam_z / fy
                    cam_y = 0     # manully set to 0 since the y axis along the vertical axis

                    height   = y_max - y_min
                    # distance = self.distance_from_camera(height)
                    distance = np.sqrt(cam_x**2 + cam_z**2)
                    angle = np.arctan2(cam_x, cam_z)

                    ### nisala added this
                    width = x_max - x_min

                    print(f"height {height}")
                    print(f"width {width}")
                    print(f"distance {distance}")
                    real_width = width*distance/height

                    print(f"real_width {real_width}")

                    # height   = y_max - y_min
                    # distance = self.distance_from_camera(height)

                    # # Angle calculations, distance corrections
                    # u                  = int((x_min + x_max) / 2)  
                    # horizontal_pixel   = u - mid_point_x
                    # angle              = self.angle_from_camera(horizontal_pixel)
                    # distance           = distance / np.cos(np.radians(angle))

                    # cam_x  = distance * math.sin(math.radians(angle))
                    # cam_y  = 0.0
                    # cam_z  = distance * math.cos(math.radians(angle))

                    lidar_x, lidar_y, lidar_z, _ = np.dot(self.lidar_to_cam_opt, [cam_x, cam_y, cam_z, 1])

                    label = f'D: {distance:.2f}m A: {angle:.2f} deg W: {real_width:.2f}'
                    cv2.putText(img, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)

                    classes.append(self.model.names[int(c)])
                    arr_x.append(lidar_x)
                    arr_y.append(lidar_y)
                    arr_width.append(real_width)

        return classes, arr_x, arr_y, arr_width
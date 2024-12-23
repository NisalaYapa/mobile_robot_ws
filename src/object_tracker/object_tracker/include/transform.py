import rclpy
import numpy as np
from scipy.spatial.transform import Rotation
from tf2_ros import TransformListener, Buffer

class GeometricTransformations():
    def __init__(self, node):
        self.node = node
        self.tf_buffer      = Buffer()
        self.tf_listener    = TransformListener(self.tf_buffer, self.node)
    
    def get_transform(self, target_frame ,source_frame):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))  
            return transform_stamped.transform

        except Exception as e:
            self.node.get_logger().warn("Failed to get transform: {}".format(e))
            return None
    
    def transform_points(self,obstacle_arr,transform):

        quaternion  = np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
        translation = np.array([transform.translation.x, transform.translation.y, transform.translation.z])

        T = np.zeros((4,4))                                      # Creating Homogeneous transformation 
        T[:3,:3] = Rotation.from_quat(quaternion).as_matrix()
        T[:3, 3] = translation.reshape(3,)
        T[ 3, 3] = 1
        transformed_points = T.dot(obstacle_arr) 
        
        return transformed_points[:2,:] 
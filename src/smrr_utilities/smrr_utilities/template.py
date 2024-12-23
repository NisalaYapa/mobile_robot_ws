import rclpy

from rclpy.node import Node
from .include.transform import GeometricTransformations

class Template(Node):  # Include the following functions within therequired class instead of this class.

    def __init__(self):
        super().__init__('template')
        self.point_arr = None # Configure this with required points to transform to new frames.
        self.transform      = GeometricTransformations(self)
        
    def transform(self):   # Template function for calling transformations.
        transformation   = self.transform.get_transform('map', 'base_link')     # Adjust the link frames here. ('target frame', 'source frame')     

        if transformation:
            transformed_points  = self.transform.transform_points(self.point_arr, transformation)
                
def main(args=None):
    
    rclpy.init(args=args)

    template = Template()

    rclpy.spin(template)

    template.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



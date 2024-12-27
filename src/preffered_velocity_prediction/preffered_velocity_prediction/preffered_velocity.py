#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import numpy as np
import random
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from filterpy.kalman import KalmanFilter
from smrr_interfaces.msg import Buffer, PrefVelocity
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

class HumanPrefferedVelocity(Node):
    def __init__(self):
        super().__init__('human_preffered_velocity')

        #self.data_reader = self.create_subscription(Float32MultiArray, 'lidar_readings', self.process_lidar_data,10)
        #self.cls_reader = self.create_subscription(Int32MultiArray, 'human_clsses', self.process_cls_data,10)
        
        # subscribe to buffer
        self.data_reader = self.create_subscription(Buffer, '/human_data_buffer/buffer', self.process_buffer_data,10)

        # publisher for preffered velocity
        self.preferred_velocity_publisher = self.create_publisher(PrefVelocity, '/preffered_velocity_prediction/preferred_velocity', 10)

        self.human_positions_pre_vel = self.create_publisher(MarkerArray, '/preffered_velocity_prediction/human_position_marker', 10)
        self.human_velocities_pre_vel = self.create_publisher(MarkerArray, '/preffered_velocity_prediction/human_velocities_marker', 10)

        self.timer = self.create_timer(0.5, self.publish_preferred_velocity)

        # class id and agent id for interface 
        self.cls_ids = []
        self.agent_ids = []
        self.preferred_velocity = []

        self.velocity_data = []
        self.cls_data = []

        self.filters = []
        self.num_of_people = 0

        # Nisala Added these things to visulization        
        self.x_position = []
        self.y_position = []
        self.x_velocity = []
        self.y_velocity = []
        
    def process_buffer_data(self, msg):
        self.agent_ids = msg.agent_ids
        self.cls_ids = msg.class_ids
        self.x_position = msg.x_positions
        self.y_position = msg.y_positions
        self.x_velocity = msg.x_mean
        self.y_velocity = msg.y_mean

        self.process_lidar_data(msg)
        self.process_cls_data(msg)

    def publish_preferred_velocity(self):
        msg = PrefVelocity()
        msg.agent_ids = self.agent_ids
        msg.preferred_velocities = self.preferred_velocity
        msg.class_ids = self.cls_ids
        self.preferred_velocity_publisher.publish(msg)
        self.get_logger().info('Publishing preferred velocity')

        self.human_position_marker()
        self.human_velocity_marker()


    def human_position_marker(self):
        marker_array = MarkerArray()  
        count = len(self.x_position)
        x_pos  = self.x_position
        y_pos = self.y_position

        for human_id in range(count):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "human_positions"
            marker.id = human_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x_pos[human_id].float_data[-1] # x position
            marker.pose.position.y = y_pos[human_id].float_data[-1]# y position
            marker.pose.position.z = 0.0  # z position (assumed flat plane)
            marker.scale.x = 0.2  # Sphere size in x
            marker.scale.y = 0.2  # Sphere size in y
            marker.scale.z = 0.01 # Sphere size in z
            marker.color.a = 1.0  # Transparency
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0  # Green
            marker.color.b = 0.0  # Blue

            # Set lifetime of the marker
            marker.lifetime = Duration(sec=1, nanosec=0)  # Marker lasts for 1 second
            marker_array.markers.append(marker)
        self.human_positions_pre_vel.publish(marker_array)
        

    def human_velocity_marker(self):
        marker_array = MarkerArray()  
        count = len(self.x_position)
        x_pos  = self.x_position
        y_pos = self.y_position
        x_vel = self.x_velocity
        y_vel = self.y_velocity
        pref_vel = self.preferred_velocity

        for human_id in range(count):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "human_velocities"
            marker.id = human_id
            marker.type = Marker.ARROW  # Change to arrow
            marker.action = Marker.ADD

            # Set the starting point of the arrow (human position)
            marker.pose.position.x = x_pos[human_id].float_data[-1]  # x position
            marker.pose.position.y = y_pos[human_id].float_data[-1] # y position
            marker.pose.position.z = 0.0  # z position (assumed flat plane)

            # Calculate the orientation of the arrow from velocity components
            vx, vy = x_vel[human_id], y_vel[human_id]  # Extract velocities from state
            velocity_magnitude = pref_vel[human_id]
            theta = np.arctan2(vy,vx)

            if velocity_magnitude > 0:
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = np.sin(theta/2)
                marker.pose.orientation.w = np.cos(theta/2)
            else:
                marker.pose.orientation.w = 1.0  # Default orientation

            # Scale the arrow: length proportional to velocity magnitude
            marker.scale.x = velocity_magnitude # Arrow length
            marker.scale.y = 0.05  # Arrow thickness
            marker.scale.z = 0.01 # Arrow thickness

            # Set the color of the arrow
            marker.color.a = 1.0  # Transparency
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0  # Green
            marker.color.b = 0.0  # Blue

            # Set lifetime of the marker
            marker.lifetime = Duration(sec=1, nanosec=0)  # Marker lasts for 1 second

            marker_array.markers.append(marker)

        self.human_velocities_pre_vel.publish(marker_array)


        

    def kalman_filter(self):
        # initialize kalman filter 
        # each human will have its own kalman filter
        kf = KalmanFilter(dim_x=2, dim_z=1)
        kf.x = np.array([[0.], [0.]])  # Start with zero speed and acceleration
        kf.F = np.array([[1., 1.], [0., 1.]])  # Constant velocity model
        kf.H = np.array([[1., 0.]])  # Measure speed only
        kf.P *= 1000.  # Initial high uncertainty of Initial Covariance Matrix
        kf.R = 1.0  # Initial measurement noise (will be updated later)
        kf.Q = np.array([[0.1, 0.0], [0.0, 0.1]])  # Process noise (will be updated later)
        return kf

    def update_filter_parameters(self, kf, class_id, std_dev, variance):
        # update the parameters of the kalman filter based on the class id
        if class_id == 'child':  # Children (typically more erratic)
            base_R = 2.0  # Base measurement noise
            base_Q = 1.5  # Base process noise factor
        elif class_id == 'normal-adult':  # Normal adults
            base_R = 1.0
            base_Q = 1.0
        elif class_id == 'elder-no-disabilities':  # Seniors (usually smoother)
            base_R = 0.8
            base_Q = 0.8
        elif class_id == 'disabled':  # People with disabilities
            base_R = 0.5
            base_Q = 0.5
        else:
            # Default values
            base_R = 1.0
            base_Q = 1.0

        # Adjust measurement noise R based on the variance (higher variance, higher noise)
        kf.R = base_R * (1 + variance / (1 + variance))  # Scale R using variance
        
        # Adjust process noise Q based on standard deviation (higher std_dev, more flexibility)
        kf.Q = np.array([[base_Q * (1 + std_dev), 0], [0, base_Q * (1 + std_dev)]])

 

    def update_kalman_filters(self, num_people):
        if num_people > len(self.filters):
            for i in range(num_people - len(self.filters)):
                self.filters.append(self.kalman_filter())
        
        elif num_people < len(self.filters):
            self.filters = self.filters[:num_people]

        self.num_of_people = num_people

        
    def process_lidar_data(self, msg):
        # x velocity data from buffer
        mean_x_velocity_data = np.array(msg.x_mean)
        mean_y_velocity_data = np.array(msg.y_mean)

        variance = np.array(msg.x_variance) + np.array(msg.y_variance)
        # assume x and y variance are independent
        std_dev = np.sqrt(variance)

        mean_speed_data = np.sqrt((mean_x_velocity_data) ** 2 + (mean_y_velocity_data) ** 2)

        
        self.velocity_data = np.vstack((mean_speed_data, std_dev, variance))

        # -1 assumes that the number of humans is unknown
        num_people = len(msg.agent_ids)
        self.get_logger().info('Received data from lidar_readings with {} people'.format(num_people))

        if num_people != self.num_of_people:
            # handles different number of people (people entering or leaving the scene)
            self.update_kalman_filters(num_people)

        # check for valid data
        if self.velocity_data is not None:
            self.compute_preffered_velocity()
        else:
            self.get_logger().info('No data received')

    def process_cls_data(self, msg):
        self.cls_data = msg.class_ids
        self.get_logger().info('Received data from human_classes')

        # check for valid data
        if self.cls_data is not None:
            self.compute_preffered_velocity()
        else:
            self.get_logger().info('No data received')

    def compute_preffered_velocity(self):

        self.get_logger().info('Computing preffered velocity')

        # Extract mean, std deviation, and variance data from the incoming array
        mean_velocity = self.velocity_data[0, :]
        std_devs = self.velocity_data[1, :]
        variances = self.velocity_data[2, :]

        preferred_velocity = [] # list of preferred velocities for each human

        # loop through each human
        for i in range (self.num_of_people):
            mean_velocity_i = mean_velocity[i]

            # handle the case where class data is not available 
            if (len(self.cls_data) -1) < i:
                class_id = '1' # default to normal adults
            else:
                class_id = self.cls_data[i]

            kf = self.filters[i]

            # update filter parameters base on class id
            self.update_filter_parameters(kf, class_id, std_devs[i], variances[i])

            # predict the next state and update
            kf.predict()
            kf.update(np.array([[mean_velocity_i]]))

            # get the preferred velocity
            preferred_velocity_i = kf.x[0, 0]
            preferred_velocity.append(preferred_velocity_i)

            self.get_logger().info(f'Agent {i} - class: {class_id}, '
                                f'Observed velocity: {mean_velocity_i:.2f}, '
                                f'Preferred velocity: {preferred_velocity_i:.2f}')
        
        # create a publisher & publish the preferred velocity
        self.preferred_velocity = preferred_velocity

def main(args=None):
    rclpy.init(args=args)
    human_preffered_velocity = HumanPrefferedVelocity()
    rclpy.spin(human_preffered_velocity)
    human_preffered_velocity.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
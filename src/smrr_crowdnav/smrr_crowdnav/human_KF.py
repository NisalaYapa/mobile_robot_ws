#!/usr/bin/env python3.8

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from smrr_interfaces.msg import Entities
import numpy as np
from builtin_interfaces.msg import Duration  # Import Duration for setting lifetime
from smrr_interfaces.msg import Entities

### This is the Kalman Filter for smoothen the huamn position and velocities
### This can visualize both raw and Kalman filterd positions and velocities in RVIZ2


class HumanKF(Node):
    def __init__(self):
        super().__init__('Human_KF_node')
        # Subscriptions
        self.create_subscription(Entities, '/laser_data_array', self.human_position_callback, 10)
        self.create_subscription(Entities, '/vel', self.human_velocity_callback, 10)
        self.create_subscription(Entities, '/goals', self.human_goal_callback, 10)

        # Publisher to output filtered positions
        self.filtered_positions_marker = self.create_publisher(MarkerArray, '/filtered_human_marker', 10)
        self.filtered_velocities_marker = self.create_publisher(MarkerArray, '/filtered_human_velocities', 10)
     
        self.positions_marker = self.create_publisher(MarkerArray, '/human_positions', 10)
        self.velocities_marker = self.create_publisher(MarkerArray, '/human_velocities', 10)

        self.filtered_positions_publisher = self.create_publisher(Entities, '/laser_data_array_kf', 10)
        self.filtered_velocity_publisher = self.create_publisher(Entities, '/vel_kf', 10)
        self.filtered_goal_publisher = self.create_publisher(Entities, '/goals_kf', 10)


        # Initialize variables
        self.human_positions = []
        self.human_velocities = []
        self.human_goals = []
        self.previous_human_count = 0

        # Kalman Filter matrices
        self.dt = 0.1 # Time step
        self.A = np.array([[1, 0, self.dt, 0],
                           [0, 1, 0, self.dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])  # State transition matrix
        self.H = np.eye(4)  # Measurement matrix
        self.Q = np.eye(4) * 0.01# Process noise covariance
        self.R = np.array([[0.5, 0, 0, 0],
                           [0, 0.5, 0, 0],
                           [0, 0, 4, 0],
                           [0, 0, 0, 4]]) # Measurement noise covariance

        # Kalman Filter state
        self.x = []  # State vectors for each human
        self.v = []  #store measurements to publish as markers
        self.P = []  # Covariance matrices for each human

        self.timer = self.create_timer(0.1, self.update_kalman_filter)

        self.get_logger().info("HumanKF Node initialized.")

    def reset_kalman_filter(self, human_count):
        """Reset the Kalman filter states and covariance matrices."""
        self.x = [np.zeros(4) for _ in range(human_count)]  # Reset state vectors

        count = min(len(self.human_positions), len(self.human_velocities))

        for i in range(count):

            # Measurement vector
            z = np.array([
                self.human_positions[i]['x'],
                self.human_positions[i]['y'],
                self.human_velocities[i]['vx'],
                self.human_velocities[i]['vy']

            ])

            self.x[i] = z
            


        self.P = [np.eye(4) for _ in range(human_count)]   # Reset covariance matrices
        self.previous_human_count = human_count

    def human_position_callback(self, msg):
        self.human_positions = [{'x': msg.x[i], 'y': msg.y[i]} for i in range(msg.count)]

        # Initialize velocities if not already done
        while len(self.human_velocities) < msg.count:
            self.human_velocities.append({'vx': 0.0, 'vy': 0.0})

        # Check if human count has changed
        if msg.count != self.previous_human_count:
            self.get_logger().info(f"Human count changed from {self.previous_human_count} to {msg.count}. Resetting Kalman filter.")
            self.reset_kalman_filter(msg.count)

        

    def human_velocity_callback(self, msg):
        self.human_velocities = [{'vx': msg.x[i], 'vy': msg.y[i]} for i in range(msg.count)]
        self.v = [np.zeros(4) for _ in range(msg.count)]

        count = min(len(self.human_positions), len(self.human_velocities))

        for i in range(count):

            # Measurement vector
            z = np.array([
                self.human_positions[i]['x'],
                self.human_positions[i]['y'],
                self.human_velocities[i]['vx'],
                self.human_velocities[i]['vy']

            ])

            self.v[i] = z

        self.publish_velocities()
        self.publish_positions()

    def human_goal_callback(self, msg):
        self.human_goals = [{'x': msg.x[i], 'y': msg.y[i]} for i in range(msg.count)]

    def update_kalman_filter(self):
        if len(self.human_velocities) < len(self.human_positions):
            self.get_logger().warning("Velocity data not yet available for all humans. Skipping update.")
            return

        for i, position in enumerate(self.human_positions):
            # Prediction step
            x_pred = self.A @ self.x[i]
            P_pred = self.A @ self.P[i] @ self.A.T + self.Q

            # Measurement vector
            z = np.array([
                position['x'],
                position['y'],
                self.human_velocities[i]['vx'],
                self.human_velocities[i]['vy']
            ])

            # Measurement update step
            y = z - self.H @ x_pred  # Measurement residual
            S = self.H @ P_pred @ self.H.T + self.R  # Residual covariance
            K = P_pred @ self.H.T @ np.linalg.inv(S)  # Kalman gain

            self.x[i] = x_pred + K @ y  # Updated state estimate
            self.P[i] = (np.eye(4) - K @ self.H) @ P_pred  # Updated covariance estimate

        # Publish filtered positions
        self.publish_filtered_positions()
        self.publish_filtered_velocities()
        

    

    def publish_filtered_positions(self):
        marker_array = MarkerArray()

        count = (len(self.x))
        vel_x = []
        vel_y = []
        pos_x = []
        pos_y = []
        goal_x = []
        goal_y = []      

        for human_id in range(len(self.x)):
            state = self.x[human_id]
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "filtered_human_positions"
            marker.id = human_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = state[0]  # x position
            marker.pose.position.y = state[1]  # y position
            marker.pose.position.z = 0.0  # z position (assumed flat plane)
            marker.scale.x = 0.2  # Sphere size in x
            marker.scale.y = 0.2  # Sphere size in y
            marker.scale.z = 0.01  # Sphere size in z
            marker.color.a = 1.0  # Transparency
            marker.color.r = 0.0  # Red
            marker.color.g = 1.0  # Green
            marker.color.b = 0.0  # Blue

            pos_x.append(state[0])
            pos_y.append(state[1])
            vel_x.append(state[2])
            vel_y.append(state[3])

            #goal_x.append(self.human_goals[human_id]['x'])
            #goal_y.append(self.human_goals[human_id]['y'])

            # Set lifetime of the marker
            marker.lifetime = Duration(sec=1, nanosec=0)  # Marker lasts for 1 second

            marker_array.markers.append(marker)

        # Publish the updated marker array
        self.pos = Entities()
        self.vel    = Entities()
        self.goals  = Entities()

        self.pos.count       = count
        self.pos.x           = pos_x
        self.pos.y           = pos_y
        
        self.vel.count     = count
        self.vel.x         = vel_x
        self.vel.y         = vel_y

        self.goals.count     = count
        self.goals.x         = goal_x
        self.goals.y         = goal_y

        self.filtered_positions_marker.publish(marker_array)
        self.filtered_positions_publisher.publish(self.pos)
        self.filtered_velocity_publisher.publish(self.vel)


 
    def publish_filtered_velocities(self):
        marker_array = MarkerArray()   

        for human_id in range(len(self.x)):
            state = self.x[human_id]
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "filtered_human_velocities"
            marker.id = human_id
            marker.type = Marker.ARROW  # Change to arrow
            marker.action = Marker.ADD

            # Set the starting point of the arrow (human position)
            marker.pose.position.x = state[0]  # x position
            marker.pose.position.y = state[1]  # y position
            marker.pose.position.z = 0.0  # z position (assumed flat plane)

            # Calculate the orientation of the arrow from velocity components
            vx, vy = state[2], state[3]  # Extract velocities from state
            velocity_magnitude = (vx**2 + vy**2)**0.5

            theta = np.arctan2(vy,vx)

            if velocity_magnitude > 0:
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = np.sin(theta/2)
                marker.pose.orientation.w = np.cos(theta/2)
            else:
                marker.pose.orientation.w = 1.0  # Default orientation

            # Scale the arrow: length proportional to velocity magnitude
            marker.scale.x = velocity_magnitude  # Arrow length
            marker.scale.y = 0.05  # Arrow thickness
            marker.scale.z = 0.01 # Arrow thickness

            # Set the color of the arrow
            marker.color.a = 1.0  # Transparency
            marker.color.r = 0.0  # Red
            marker.color.g = 1.0  # Green
            marker.color.b = 0.0  # Blue

            # Set lifetime of the marker
            marker.lifetime = Duration(sec=1, nanosec=0)  # Marker lasts for 1 second

            marker_array.markers.append(marker)

        self.filtered_velocities_marker.publish(marker_array)


    def publish_positions(self):
        marker_array = MarkerArray()   

        for human_id in range(len(self.v)):
            state = self.v[human_id]
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "filtered_human_velocities"
            marker.id = human_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = state[0]  # x position
            marker.pose.position.y = state[1]  # y position
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

        self.positions_marker.publish(marker_array)



    def publish_velocities(self):
        marker_array = MarkerArray()   

        for human_id in range(len(self.v)):
            state = self.v[human_id]
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "human_velocities"
            marker.id = human_id
            marker.type = Marker.ARROW  # Change to arrow
            marker.action = Marker.ADD

            # Set the starting point of the arrow (human position)
            marker.pose.position.x = state[0]  # x position
            marker.pose.position.y = state[1]  # y position
            marker.pose.position.z = 0.0  # z position (assumed flat plane)

            # Calculate the orientation of the arrow from velocity components
            vx, vy = state[2], state[3]  # Extract velocities from state
            velocity_magnitude = (vx**2 + vy**2)**0.5
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

        self.filtered_velocities_marker.publish(marker_array)

    def plot_x(self):
        pass



def main(args=None):
    rclpy.init(args=args)
    node = HumanKF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

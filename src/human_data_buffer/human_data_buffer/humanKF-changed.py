#!/usr/bin/env python3.8

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from smrr_interfaces.msg import Entities
import numpy as np
from builtin_interfaces.msg import Duration  # Import Duration for setting lifetime
from smrr_interfaces.msg import Entities, VelocityClassData

### This is the Kalman Filter for smoothen the huamn position and velocities
### This can visualize both raw and Kalman filterd positions and velocities in RVIZ2


class HumanKF(Node):
    def __init__(self):
        super().__init__('Human_KF_vel_node')

        # Subscriptions
        self.create_subscription(VelocityClassData, '/human_data_buffer/velocity_class_data', self.human_callback, 10)
        self.filtered_human_publisher = self.create_publisher(VelocityClassData, 'human_data_buffer/velocity_class_data_kf', 10)



        # Initialize variables
        self.human_positions = []
        self.human_velocities = []
        self.class_id = []
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

        #self.timer = self.create_timer(0.1, self.update_kalman_filter)

        self.get_logger().info("HumanKF Node initialized.")

    def reset_kalman_filter(self, human_count):
        try:
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
                


            self.P = [np.eye(4) for _ in range(human_count)]   #Reset covariance matrices
            self.previous_human_count = human_count
        except:
            pass

    def human_callback(self, msg):
        try:
            self.change = 0
            count = len(msg.x_positions)
            print(msg.class_ids)
            print(count)
            self.class_id = msg.class_ids
            self.human_positions = [{'x': msg.x_positions[i], 'y': msg.y_positions[i]} for i in range(count)]
            self.human_velocities = [{'vx': msg.x_velocities[i], 'vy': msg.y_velocities[i]} for i in range(count)]
            print(f"###################### {self.human_velocities} #####################")


            if "-1" in self.class_id:
                self.change = 1

            #Check if human count has changed
            if count != self.previous_human_count:
                self.get_logger().info(f"Human count changed from {self.previous_human_count} to {count}. Resetting Kalman filter.")
                self.reset_kalman_filter(count)

            elif self.change:
                self.filtered_human_publisher.publish(msg)

                

            else:
                self.update_kalman_filter()
        except:
            pass

        

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
            self.publish_human_data()



        
    def publish_human_data(self):

        vel_x = []
        vel_y = []
        pos_x = []
        pos_y = []   

        for human_id in range(len(self.x)):
            state = self.x[human_id]

            pos_x.append(state[0])
            pos_y.append(state[1])
            vel_x.append(np.clip(state[2], -2.0, 2.0))
            vel_y.append(np.clip(state[3], -2.0, 2.0))
    

        self.human_kf = VelocityClassData()


        self.human_kf.class_ids = self.class_id
        self.human_kf.x_positions = pos_x
        self.human_kf.y_positions  = pos_y
        self.human_kf.x_velocities = vel_x
        self.human_kf.y_velocities = vel_y

        


                     
        self.filtered_human_publisher.publish(self.human_kf)


 

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
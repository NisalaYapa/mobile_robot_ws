#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from smrr_interfaces.msg import Entities, VelocityClassData
import numpy as np

class VelocityExtractor(Node):
    def __init__(self):
        super().__init__('velocity_extractor')

        # subscribe to x position, y position, and class topics
        self.susbscription = self.create_subscription(
            Entities,
            '/object_tracker/laser_data_array',
            self.callback_velocity_input,
            10
            )
        
        # publisher for velocity data
        # self.pub_x_velocity = self.create_publisher(Float32MultiArray, 'x_velocity', 10)
        # self.pub_y_velocity = self.create_publisher(Float32MultiArray, 'y_velocity', 10)
        # self.pub_class = self.create_publisher(Int32MultiArray, 'human_classes', 10)
        
        # custom interface
        self.pub_velocity_class = self.create_publisher(VelocityClassData, 'velocity_class_data', 10)

        # storage for previous data
        self.prev_x = {}
        self.prev_y = {}

        # update rate
        self.update_rate = 0.5 # 2 Hz

    def callback_velocity_input(self, msg):
        x_positions  = msg.x #list of x positions
        y_positions = msg.y #list of y positions
        class_ids = msg.classes
        agent_count = msg.count

        # calculate velocity and update previous positions
        x_vel = []
        y_vel = []
        class_list = []

        for i, (x_position, y_position, class_id) in enumerate(zip(x_positions, y_positions, class_ids)):
            pre_x = self.prev_x.get(i, x_position)
            pre_y = self.prev_y.get(i, y_position)

            try:
                # if agent left
                if pre_x == 0.0 or pre_y == 0.0:
                    vx = -100.0
                    vy = -100.0
                    cl_id = "-1"
                    # remove agent from previous positions
                    self.prev_x.pop(i, None)
                    self.prev_y.pop(i, None)

                else:
                    vx = (x_position - pre_x) / self.update_rate
                    vy = (y_position - pre_y) / self.update_rate
                    cl_id = class_id
            except:
                vx = 0.0
                vy = 0.0
                cl_id = '0'
                self.get_logger().info('Error calculating velocity in callback_velocity_input')


            # update previous positions
            self.prev_x[i] = x_position
            self.prev_y[i] = y_position

            x_vel.append(vx)
            y_vel.append(vy)
            class_list.append(cl_id)

        # publish the velocity data
        self.publish_velocity(x_vel, y_vel, class_list, x_positions, y_positions)

    def publish_velocity(self, x_vel,y_vel, class_list, x_positions, y_positions):
        # publish x velocity
        # msg = Float32MultiArray()
        # msg.data = x_vel
        # self.pub_x_velocity.publish(msg)

        # # publish y velocity
        # msg = Float32MultiArray()
        # msg.data = y_vel
        # self.pub_y_velocity.publish(msg)

        # # publish class data
        # msg = Int32MultiArray()
        # msg.data = class_list
        # self.pub_class.publish(msg)

        # custom interface
        msg = VelocityClassData()
        msg.x_velocities = x_vel
        msg.y_velocities = y_vel
        msg.class_ids = class_list
        msg.x_positions = x_positions
        msg.y_positions = y_positions
        self.pub_velocity_class.publish(msg)


        # loging the published data
        self.get_logger().info(f'Published x velocity: {x_vel}')
        self.get_logger().info(f'Published y velocity: {y_vel}')
        self.get_logger().info(f'Published class data: {class_list}')
        self.get_logger().info(f'Published x positions: {x_positions}')
        self.get_logger().info(f'Published y positions: {y_positions}')

def main(args=None):
    rclpy.init(args=args)

    velocity_extractor = VelocityExtractor()

    rclpy.spin(velocity_extractor)

    velocity_extractor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

        
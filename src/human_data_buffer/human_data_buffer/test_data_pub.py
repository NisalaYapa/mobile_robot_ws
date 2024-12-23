#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from smrr_interfaces.msg import Entities, VelocityClassData
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import numpy as np
import random

# create a publisher in Entity data type for testing 
class TestDataPub(Node):
    def __init__(self):
        super().__init__('test_data_pub')
        self.pub = self.create_publisher(Entities, '/object_tracker/laser_data_array', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.clases = []
        self.x_positions = []
        self.y_positions = []

    def agennt_entry(self):
        new_class = str(random.randint(0, 3))
        new_x =random.uniform(1.0,10.0)
        new_y = random.uniform(1.0,10.0)

        self.clases.append(new_class)
        self.x_positions.append(new_x)
        self.y_positions.append(new_y)

        self.get_logger().info('New agent: class: {new_class}, x: {new_x}, y: {new_y}')
    def agent_exit(self):
        if self.x_positions:
            inx = random.randint(0,len(self.x_positions)-1)
            self.x_positions[inx] = 0.0
            self.y_positions[inx] = 0.0

            self.get_logger().info(' Agent exit : class: {self.clases[inx]}, x: {self.x_positions[inx]}, y: {self.y_positions[inx]}')

            # publish the data
            self.publish_data()

            # remove the agent in next iteration
            del self.clases[inx]
            del self.x_positions[inx]
            del self.y_positions[inx]


    def timer_callback(self):
        flag = 0
        if random.random() < 0.4:
            # 20 % chance of agent enter
            self.agennt_entry()
            flag = 1
        if random.random() < 0.38 and self.x_positions:
            # 15 % chance of agent exit
            self.agent_exit()
            flag = 1

        if random.random() < 0.95 and self.x_positions and flag == 0:
            # 95 % chance of agent move
            self.update_positions()
        
        self.publish_data()

    def update_positions(self):
        for i in range(len(self.x_positions)):
            if self.x_positions[i] == 0 or self.y_positions[i] == 0:
                continue
            self.x_positions[i] += random.uniform(-0.2,0.2)
            self.y_positions[i] += random.uniform(-0.2,0.2)

    def publish_data(self):
        msg = Entities()
        msg.x = self.x_positions
        msg.y = self.y_positions
        msg.classes = self.clases
        msg.count = len(self.x_positions)
        self.pub.publish(msg)

        self.get_logger().info(f'Published {msg.count} agents , x: {msg.x}, y: {msg.y}, classes: {msg.classes}')
        

def main(args=None):
    rclpy.init(args=args)

    test_data_pub = TestDataPub()

    rclpy.spin(test_data_pub)

    test_data_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



        

 


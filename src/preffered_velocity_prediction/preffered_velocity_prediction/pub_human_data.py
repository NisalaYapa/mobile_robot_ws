#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import numpy as np
import random
from std_msgs.msg import Float32MultiArray, Int32MultiArray

class ArrayPublisher(Node):

    def __init__(self):
        super().__init__('human_data_publisher')
        self.data_publisher = self.create_publisher(Float32MultiArray, 'lidar_readings', 10)
        self.timer = self.create_timer(0.5, self.publish_human_data)

        self.cls_data_publisher = self.create_publisher(Int32MultiArray, 'human_clsses', 10)
        self.cls_timer = self.create_timer(8, self.publish_human_classes) 
        # update the human classes every 5 seconds

    def publish_human_data(self):
        num_of_people = 8 # number of people (fixed for now)
        buffer_data = np.random.uniform(0,1.4,(3,num_of_people)).astype(float)

        # array for human data
        human_data = buffer_data.flatten().tolist()

        msg = Float32MultiArray()
        msg.data = human_data

        self.get_logger().info('Publishing human data to lidar_readings')
        self.data_publisher.publish(msg)

    def publish_human_classes(self):
        num_of_people = 8
        cls_list = [random.randint(0,3) for i in range (num_of_people)]
        cls_msg = Int32MultiArray()
        cls_msg.data = cls_list
        self.get_logger().info('Publishing human classes to human_clases')
        self.cls_data_publisher.publish(cls_msg)


def main(args=None):
    rclpy.init(args=args)
    array_publisher = ArrayPublisher()
    rclpy.spin(array_publisher)
    array_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
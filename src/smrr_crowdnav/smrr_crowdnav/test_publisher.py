#!/usr/bin/env python3.8

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from smrr_interfaces.msg import Entities
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import random
from time import sleep
import math

# Publisher Node
class TestPublisherNode(Node):
    def __init__(self):
        super().__init__('test_publisher_node')

        # Publishers for the topics
        self.human_position_pub = self.create_publisher(Entities, 'human_position', 10)
        self.human_velocity_pub = self.create_publisher(Entities, 'human_velocity', 10)
        self.human_goal_pub = self.create_publisher(Entities, 'human_goal', 10)
        self.robot_position_pub = self.create_publisher(Odometry, 'robot_position', 10)
        self.robot_velocity_pub = self.create_publisher(Twist, 'robot_velocity', 10)
        self.robot_goal_pub = self.create_publisher(Float32MultiArray, 'robot_goal', 10)

        # Timer to publish sample data every second
        self.timer = self.create_timer(0.5, self.publish_sample_data)

        self.get_logger().info("Test Publisher Node has been started")

    def publish_sample_data(self):
        # Generate random data for human agents
        num_agents = 3
        human_pos_msg = Entities()
        human_pos_msg.count = num_agents
        human_pos_msg.x = [random.uniform(-5, 5) for _ in range(num_agents)]
        human_pos_msg.y = [random.uniform(-5, 5) for _ in range(num_agents)]

        human_vel_msg = Entities()
        human_vel_msg.count = num_agents
        human_vel_msg.x = [random.uniform(-1, 1) for _ in range(num_agents)]
        human_vel_msg.y = [random.uniform(-1, 1) for _ in range(num_agents)]

        human_goal_msg = Entities()
        human_goal_msg.count = num_agents
        human_goal_msg.x = [random.uniform(0, 10) for _ in range(num_agents)]
        human_goal_msg.y = [random.uniform(0, 10) for _ in range(num_agents)]

        # Publish human states
        self.human_position_pub.publish(human_pos_msg)
        self.human_velocity_pub.publish(human_vel_msg)
        self.human_goal_pub.publish(human_goal_msg)

        # Generate and publish random robot odometry (position and orientation)
        robot_pos_msg = Odometry()
        robot_pos_msg.pose.pose.position.x = random.uniform(-5, 5)
        robot_pos_msg.pose.pose.position.y = random.uniform(-5, 5)
        yaw = random.uniform(-math.pi, math.pi)  # Random orientation in radians
        robot_pos_msg.pose.pose.orientation = self.quaternion_from_euler(0, 0, yaw)

        self.robot_position_pub.publish(robot_pos_msg)

        # Generate and publish random robot velocity
        robot_vel_msg = Twist()
        robot_vel_msg.linear.x = random.uniform(0, 1)  # Linear velocity
        robot_vel_msg.angular.z = random.uniform(-1, 1)  # Angular velocity (omega)
        self.robot_velocity_pub.publish(robot_vel_msg)


        # Log the published data
        self.get_logger().info(f"Published human position: {human_pos_msg.x}, {human_pos_msg.y}")
        self.get_logger().info(f"Published human velocity: {human_vel_msg.x}, {human_vel_msg.y}")
        self.get_logger().info(f"Published human goal: {human_goal_msg.x}, {human_goal_msg.y}")
        self.get_logger().info(f"Published robot position: ({robot_pos_msg.pose.pose.position.x}, {robot_pos_msg.pose.pose.position.y}) with yaw: {yaw}")
        self.get_logger().info(f"Published robot velocity: {robot_vel_msg.linear.x}, {robot_vel_msg.angular.z}")
        

    def quaternion_from_euler(self, roll, pitch, yaw):
        """ Convert Euler angles to quaternion for odometry message """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Odometry().pose.pose.orientation.__class__(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    test_publisher_node = TestPublisherNode()

    try:
        rclpy.spin(test_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

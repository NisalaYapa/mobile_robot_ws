#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from smrr_interfaces.msg import Entities, VelocityClassData
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

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
        self.raw_human_position = self.create_publisher(MarkerArray, '/human_data_buffer/raw_positions', 10)

        self.pub_velocity_class = self.create_publisher(VelocityClassData, '/human_data_buffer/velocity_class_data', 10)
        self.human_position_publisher = self.create_publisher(MarkerArray, '/human_data_buffer/positions_latest', 10)
        self.human_velocity_publisher = self.create_publisher(MarkerArray, '/human_data_buffer/velocities_latest', 10)

        # storage for previous data
        self.prev_x = []
        self.prev_y = []


        # update rate
        self.update_rate = 0.5 # 2 Hz

    def callback_velocity_input(self, msg):
        x_positions  = msg.x #list of x positions
        y_positions = msg.y #list of y positions
        class_ids = msg.classes
        agent_count = msg.count
        print(x_positions)
        print(y_positions)

       
        self.raw_human_position_marker(msg)

        # calculate velocity and update previous positions
        x_vel = []
        y_vel = []
        
        class_list = []

        for i, (x_position, y_position, class_id) in enumerate(zip(x_positions, y_positions, class_ids)):
                      

            try:
           
                # if agent left
                #if pre_x == 0.0 or pre_y == 0.0:
                if y_position == 0.0:
                    vx = 0.0
                    vy = 0.0
                    cl_id = "-1"
                    self.get_logger().warning("removed a person")
                    # remove agent from previous positions
                    self.prev_x.pop(i)
                    self.prev_y.pop(i)


                else:
                    if i + 1 <= len(self.prev_x):
                        pre_x = self.prev_x[i]
                        pre_y = self.prev_y[i]
                        vx = (x_position - pre_x) / self.update_rate
                        vy = (y_position - pre_y) / self.update_rate
                        cl_id = class_id



                    else:
                        pre_x = x_position
                        pre_y = y_position
                        self.get_logger().warning("added new person")      
                            
                        vx = 0.0
                        vy = 0.0
                        cl_id = class_id

            except:
                vx = 0.0
                vy = 0.0
                cl_id = '0'
                self.get_logger().info('Error calculating velocity in callback_velocity_input')


            # update previous positions
            # self.prev_x[i] = x_position
            # self.prev_y[i] = y_position

            # list based method
            if i < len(self.prev_x):
                self.prev_x[i] = x_position
                self.prev_y[i] = y_position
            else:
                self.prev_x.append(x_position)
                self.prev_y.append(y_position)

            x_vel.append(vx)
            y_vel.append(vy)
            class_list.append(cl_id)

        # publish the velocity data
        self.publish_velocity(x_vel, y_vel, class_list, x_positions, y_positions)

    def publish_velocity(self, x_vel,y_vel, class_list, x_positions, y_positions):

        # custom interface
        msg = VelocityClassData()
        msg.x_velocities = x_vel
        msg.y_velocities = y_vel
        msg.class_ids = class_list
        msg.x_positions = x_positions
        msg.y_positions = y_positions
        self.pub_velocity_class.publish(msg)

        self.publish_latest_positions(msg)
        self.publish_latest_velocities(msg)

        print(f"x velocities {x_vel}")
        print(f"y velocities {y_vel}")

        # loging the published data
        # self.get_logger().info(f'Published x velocity: {x_vel}')
        # self.get_logger().info(f'Published y velocity: {y_vel}')
        # self.get_logger().info(f'Published class data: {class_list}')
        # self.get_logger().info(f'Published x positions: {x_positions}')
        # self.get_logger().info(f'Published y positions: {y_positions}')


    def publish_latest_positions(self, msg):
        marker_array = MarkerArray()  
        count = len(msg.x_positions) 
        x_pos  = msg.x_positions
        y_pos = msg.y_positions
        #self.get_logger(f"human_count {count}")

        colors = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0),  # Blue
            (1.0, 1.0, 0.0),  # Yellow
            (0.0, 1.0, 1.0),  # Cyan
            (1.0, 0.0, 1.0),  # Magenta
            (0.5, 0.5, 0.5),  # Grey
            (1.0, 0.5, 0.0),  # Orange
            (0.5, 0.0, 1.0),  # Purple
            (0.0, 0.5, 1.0),  # Light Blue
        ]

        for human_id in range(count):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "human_positions"
            marker.id = human_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x_pos[human_id] # x position
            marker.pose.position.y = y_pos[human_id] # y position
            marker.pose.position.z = 0.0  # z position (assumed flat plane)
            marker.scale.x = 0.2  # Sphere size in x
            marker.scale.y = 0.2  # Sphere size in y
            marker.scale.z = 0.01 # Sphere size in z
            marker.color.a = 1.0  # Transparency

            color = colors[3]
            marker.color.r = color[0] # Red
            marker.color.g = color[1] # Green
            marker.color.b = color[2]  # Blue

            # Set lifetime of the marker
            marker.lifetime = Duration(sec=1, nanosec=0)  # Marker lasts for 1 second
            marker_array.markers.append(marker)
        self.human_position_publisher.publish(marker_array)
        

    def publish_latest_velocities(self, msg):
        marker_array = MarkerArray()  
        count = len(msg.x_positions) 
        x_pos  = msg.x_positions
        y_pos = msg.y_positions
        x_vel = msg.x_velocities
        y_vel = msg.y_velocities


        colors = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0),  # Blue
            (1.0, 1.0, 0.0),  # Yellow
            (0.0, 1.0, 1.0),  # Cyan
            (1.0, 0.0, 1.0),  # Magenta
            (0.5, 0.5, 0.5),  # Grey
            (1.0, 0.5, 0.0),  # Orange
            (0.5, 0.0, 1.0),  # Purple
            (0.0, 0.5, 1.0),  # Light Blue
        ]

        for human_id in range(count):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "human_velocities"
            marker.id = human_id
            marker.type = Marker.ARROW  # Change to arrow
            marker.action = Marker.ADD

            # Set the starting point of the arrow (human position)
            marker.pose.position.x = x_pos[human_id]  # x position
            marker.pose.position.y = y_pos[human_id] # y position
            marker.pose.position.z = 0.0  # z position (assumed flat plane)

            # Calculate the orientation of the arrow from velocity components
            vx, vy = x_vel[human_id], y_vel[human_id]  # Extract velocities from state
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
            color = colors[3]
            marker.color.r = color[0] # Red
            marker.color.g = color[1] # Green
            marker.color.b = color[2]  # Blue

            # Set lifetime of the marker
            marker.lifetime = Duration(sec=0, nanosec=0)  # Marker lasts for 1 second

            marker_array.markers.append(marker)

        self.human_velocity_publisher.publish(marker_array)

    def raw_human_position_marker(self, msg):
        marker_array = MarkerArray()  
        count = len(msg.x) 
        x_pos  = msg.x
        y_pos = msg.y

        for human_id in range(count):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "human_positions"
            marker.id = human_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x_pos[human_id] # x position
            marker.pose.position.y = y_pos[human_id] # y position
            marker.pose.position.z = 0.0  # z position (assumed flat plane)
            marker.scale.x = 0.2  # Sphere size in x
            marker.scale.y = 0.2  # Sphere size in y
            marker.scale.z = 0.01 # Sphere size in z
            marker.color.a = 1.0  # Transparency
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0  # Green
            marker.color.b = 0.0  # Blue

            # Set lifetime of the marker
            marker.lifetime = Duration(sec=0, nanosec=0)  # Marker lasts for 1 second
            marker_array.markers.append(marker)
        self.raw_human_position.publish(marker_array)



def main(args=None):
    rclpy.init(args=args)

    velocity_extractor = VelocityExtractor()

    rclpy.spin(velocity_extractor)

    velocity_extractor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

        
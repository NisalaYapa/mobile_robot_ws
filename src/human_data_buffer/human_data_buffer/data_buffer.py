#!/usr/bin/env python3

# buffer with statistics for each agent in the multi-dimensional numpy array

import rclpy
from rclpy.node import Node
from smrr_interfaces.msg import VelocityClassData, Buffer, DataElementFloat, DataElementString
import numpy as np
from collections import deque, Counter
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration



class DataBufferNode(Node):
    def __init__(self):
        super().__init__('data_buffer_node')

        # New subscription for the velocity data
        self.subscription = self.create_subscription(
            VelocityClassData,
            '/human_data_buffer/velocity_class_data_kf',
            self.callback_velocity_data,
            10
        )
        # Publisher for the buffer data
        # The buffer message contains lists of data for each agent
        self.pub_buffer = self.create_publisher(Buffer, '/human_data_buffer/buffer', 10)
        self.pub_timer = self.create_timer(0.5, self.publish_buffer)
        self.human_position_buffer = self.create_publisher(MarkerArray, '/human_data_buffer/human_position_buffer', 10)
        self.human_velocity_buffer = self.create_publisher(MarkerArray, '/human_data_buffer/human_velocity_buffer', 10)

        # Initialize the buffer matrix with a numpy array
        # Structure: each row is [human_id, x_velocities (deque), y_velocities (deque), class_details (deque), x_positions(deque), y_positions(deque), statistics (dict)]
        self.agent_matrix = np.empty((0, 7), dtype=object)
        self.next_available_id = 0  # Tracks the next unique ID for new agents
        self.reset_threshold = 1000  # Reset IDs if they exceed this value

    def callback_velocity_data(self, msg):
        x_velocities = msg.x_velocities  # List of x velocities from the message
        y_velocities = msg.y_velocities  # List of y velocities from the message
        x_positions = msg.x_positions    # List of x positions from the message
        y_positions = msg.y_positions    # List of y positions from the message
        class_ids = msg.class_ids        # List of class IDs from the message

        try:
            # Process each agent's data from the message
            for i, (x_vel, y_vel, class_id, x_position, y_position) in enumerate(zip(x_velocities, y_velocities, class_ids, x_positions, y_positions)):
                # Check if the agent has left (indicated by -1 velocities)

                if class_id == '-1' :
                    # If agent has left, remove their row from the buffer
                    self.get_logger().warn("Agent has left")
                    try:
                        self.remove_agent(i)
                    except Exception as e:
                        self.get_logger().error(f"Error removing agent from buffer: {e}")
                    continue

                # Update data for existing agent or add new agent
                if i < len(self.agent_matrix):
                    # Existing agent - update their data
                    try:
                        self.update_existing_agent(i, x_vel, y_vel, class_id, x_position, y_position)
                    except Exception as e:
                        self.get_logger().error(f"Error updating existing agent in buffer: {e}")
                else:
                    # New agent - add a new row for this agent
                    try:
                        self.add_new_agent(x_vel, y_vel, class_id, x_position, y_position)

                    except Exception as e:
                        self.get_logger().error(f"Error adding new agent to buffer: {e}")

                # Calculate and store statistics after updating each agent
                try:
                    self.calculate_statistics(i)
                except Exception as e:
                    self.get_logger().error(f"Error calculating statistics for agent in buffer: {e}")

                
                # Log buffer status for debugging
                try:
                    #self.log_buffer_status()
                    pass
                except Exception as e:
                    self.get_logger().error(f"Error logging buffer status in the loop: {e}")

        except Exception as e:
            self.get_logger().error(f"Error assigning data to buffer in the loop: {e}")

    def add_new_agent(self, x_vel, y_vel, class_id, x_position, y_position):
        # Initialize lists with a max of 10 entries each for velocities and class details, plus a dictionary for statistics
        new_row = [
            self.next_available_id,      # Assign the next available ID
            deque([x_vel], maxlen=10),   # x velocities deque
            deque([y_vel], maxlen=10),   # y velocities deque
            deque([class_id], maxlen=10), # class details deque
            deque([x_position], maxlen=10), # x positions deque
            deque([y_position], maxlen=10), # y positions deque
            {"y_variance": 0}                           # Placeholder for statistics dictionary
        ]
        print(f"new_row {new_row}")
        self.agent_matrix = np.vstack([self.agent_matrix, new_row])  # Add new row to the numpy matrix
        self.next_available_id += 1  # Increment ID for the next new agent

        # Check if ID threshold is reached and reset if necessary
        if self.next_available_id > self.reset_threshold:
            self.reset_ids()

    def update_existing_agent(self, index, x_vel, y_vel, class_id, x_position, y_position):
        # Append new data to existing agent's deques
        if index >= len(self.agent_matrix):
            self.get_logger().warning(f"Tried to update existing agent for index {index}, out of bounds.")
            return
        
        self.agent_matrix[index, 1].append(x_vel)  # Update x velocities
        self.agent_matrix[index, 2].append(y_vel)  # Update y velocities
        self.agent_matrix[index, 3].append(class_id)  # Update class IDs
        self.agent_matrix[index, 4].append(x_position)  # Update x positions
        self.agent_matrix[index, 5].append(y_position)  # Update y positions

    def calculate_statistics(self, index):
        
        if index >= len(self.agent_matrix):
            self.get_logger().warning(f"Tried to calculate statistics for index {index},out of bounds.")
            return

        # Calculate statistics for the agent's velocities and class data
        x_vals = np.array(self.agent_matrix[index, 1])  # Convert x velocities to numpy array
        y_vals = np.array(self.agent_matrix[index, 2])  # Convert y velocities to numpy array
        class_vals = list(self.agent_matrix[index, 3])   # Convert class IDs to a list

        # Compute mean, standard deviation, and variance for x and y velocities
        x_mean = np.mean(x_vals)
        y_mean = np.mean(y_vals)
        x_std_dev = np.std(x_vals)
        y_std_dev = np.std(y_vals)
        x_variance = np.var(x_vals)
        y_variance = np.var(y_vals)

        # Determine the majority class ID (most common value)
        majority_class_id = Counter(class_vals).most_common(1)[0][0]

        # Store the calculated statistics in the agent's statistics dictionary
        self.agent_matrix[index, 6] = {
            "x_mean": x_mean,
            "y_mean": y_mean,
            "x_std_dev": x_std_dev,
            "y_std_dev": y_std_dev,
            "x_variance": x_variance,
            "y_variance": y_variance,
            "majority_class_id": majority_class_id
        }
    
    def publish_buffer(self):
        msg = Buffer()  # Create a new Buffer message

        # nested structure
        agent_ids = []
        x_velocities = []
        y_velocities = []
        class_ids = []
        x_positions = []
        y_positions = []
        x_mean = []
        y_mean = []
        x_std_dev = []
        y_std_dev = []
        x_variance = []
        y_variance = []
        majority_class_ids = []


        for i in range(len(self.agent_matrix)):
            agent_ids.append(self.agent_matrix[i, 0])
            x_vel_list = DataElementFloat() # create a new DataElementFloat object for each agent to store x velocities of the agent
            y_vel_list = DataElementFloat()
            class_list = DataElementString()
            x_pos_list = DataElementFloat()
            y_pos_list = DataElementFloat()

            x_vel_list.float_data = list(self.agent_matrix[i, 1])
            y_vel_list.float_data = list(self.agent_matrix[i, 2])
            class_list.string_data = list(self.agent_matrix[i, 3])
            x_pos_list.float_data = list(self.agent_matrix[i, 4])
            y_pos_list.float_data = list(self.agent_matrix[i, 5])

            print(f"class ids {class_list}")
            x_velocities.append(x_vel_list)
            y_velocities.append(y_vel_list)
            class_ids.append(class_list)
            x_positions.append(x_pos_list)
            y_positions.append(y_pos_list)


            # Get statistics from the statistics dictionary
            stats = self.agent_matrix[i, 6]
            x_mean.append(stats.get('x_mean', 0))
            y_mean.append(stats.get('y_mean', 0))
            x_std_dev.append(stats.get('x_std_dev', 0))
            y_std_dev.append(stats.get('y_std_dev', 0))
            x_variance.append(stats.get('x_variance', 0))
            y_variance.append(stats.get('y_variance', 0))
            majority_class_ids.append(stats.get('majority_class_id', -1))

        #print("x_velocities")
        #print(x_velocities)

        # Assign the formatted data to the message fields
        msg.agent_count = int(len(agent_ids))
        msg.agent_ids = agent_ids
        msg.x_velocities = x_velocities
        msg.y_velocities = y_velocities
        msg.class_ids = class_ids
        msg.x_positions = x_positions
        msg.y_positions = y_positions
        msg.x_mean = x_mean
        msg.y_mean = y_mean
        msg.x_std_dev = x_std_dev
        msg.y_std_dev = y_std_dev
        msg.x_variance = x_variance
        msg.y_variance = y_variance
        msg.majority_class_id = majority_class_ids
        # publish the message
        self.pub_buffer.publish(msg)
        self.get_logger().info("Published buffer data.")

        self.human_position_marker(msg)
        self.human_velocity_marker(msg)

        # log buffer x and y velocities
        #self.get_logger().info(f"x_velocities: {x_velocities}")
        #self.get_logger().info(f"y_velocities: {y_velocities}")

    def remove_agent(self, index):
        # Remove the row corresponding to the agent that has left
        if index < len(self.agent_matrix):
            removed_id = self.agent_matrix[index, 0]
            self.agent_matrix = np.delete(self.agent_matrix, index, axis=0)
            self.get_logger().info(f'Removed agent with ID {removed_id} from buffer')

    def reset_ids(self):
        # Reset all IDs to values from 0 up to the current number of agents
        for idx in range(len(self.agent_matrix)):
            self.agent_matrix[idx, 0] = idx  # Assign IDs starting from 0
        self.next_available_id = len(self.agent_matrix)  # Set the next ID after the last used ID
        self.get_logger().info("Reset all human IDs in the buffer.")

    def log_buffer_status(self):
        # Log the current status of the buffer for debugging
        for row in self.agent_matrix:
            stats = row[6] if row[6] else {}  # Access the statistics column
            self.get_logger().info(f'Agent ID: {row[0]}, X Velocities: {list(row[1])}, '
                                   f'Y Velocities: {list(row[2])}, Class Details: {list(row[3])}, '
                                   f'X Positions: {list(row[4])}, Y Positions: {list(row[5])}, '
                                   f'Statistics: {stats}')

    def get_buffer_for_agent(self, agent_id):
        # Retrieve buffer data for a specific agent by ID
        agent_row = self.agent_matrix[self.agent_matrix[:, 0] == agent_id]
        return agent_row if agent_row.size else None

    def get_all_buffers(self):
        # Retrieve buffer data for all agents
        return self.agent_matrix

    def human_position_marker(self, msg):
        marker_array = MarkerArray()  
        count = msg.agent_count
        x_pos  = msg.x_positions
        y_pos = msg.y_positions

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
            marker.lifetime = Duration(sec=0, nanosec=0)  # Marker lasts for 1 second
            marker_array.markers.append(marker)
        self.human_position_buffer.publish(marker_array)
        

    def human_velocity_marker(self, msg):
        marker_array = MarkerArray()  
        count = len(msg.x_positions) 
        x_pos  = msg.x_positions
        y_pos = msg.y_positions
        x_vel = msg.x_mean
        y_vel = msg.y_mean

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
            marker.lifetime = Duration(sec=0, nanosec=0)  # Marker lasts for 1 second

            marker_array.markers.append(marker)

        self.human_velocity_buffer.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    data_buffer_node = DataBufferNode()
    rclpy.spin(data_buffer_node)
    data_buffer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
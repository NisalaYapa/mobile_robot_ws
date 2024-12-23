#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from smrr_interfaces.action import NavigateToGoal
import argparse
import yaml
import os


### This is the action client for control nodes (action servers)
### the goal should be given as parameters ( --x 10.0 --y 5.8 )

class NavigateToGoalClient(Node):
    def __init__(self):
        super().__init__('navigate_to_goal_client')

        # Create an action client for 'NavigateToGoal'
        self._action_client = ActionClient(self, NavigateToGoal, 'navigate_to_goal')

        # Store the goal handle for future use (cancellation, etc.)
        self.goal_handle = None

        # Load the YAML config file
        package_path = os.path.dirname(__file__)  # Current file's directory
        config_path = os.path.join(package_path,'config', 'scenario_config.yaml')
        
        with open(config_path, 'r') as file:
            configs = yaml.safe_load(file)
        
        # Get parameters for this class
        node_name = "GoalClient"  # Define your node's name
        node_configs = configs.get(node_name, {})

        # Set class attributes for each parameter
        for key, value in node_configs.items():
            setattr(self, key, value)  # Dynamically add attributes

        # Log the loaded parameters
        goal = getattr(self, 'goal', (0.0,0.0))  # Default to 1 if not defined
        print(f"Loaded goal: {goal}")
        
        # Environment-related variables
        self.goal = goal        

    def send_goal(self, x, y):
        """Send a goal to the action server."""
        goal_msg = NavigateToGoal.Goal()
        goal_msg.goal_x = x
        goal_msg.goal_y = y

        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Send the goal and set up callbacks for feedback and goal response
        self.get_logger().info(f'Sending goal to coordinates: x={x}, y={y}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response from the action server after sending the goal."""
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected by server.')
            return

        self.get_logger().info('Goal accepted by server.')
        # Wait for the result once the goal is processed by the server
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: Distance to goal = {feedback.distance_to_goal}')

    def get_result_callback(self, future):
        """Handle the result after goal execution is complete."""
        result = future.result().result

        if result.success:
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().info('Failed to reach the goal.')

        # Reset the goal handle after processing the result
        self.goal_handle = None


def main(args=None):
    """Main function to run the action client."""
    rclpy.init(args=args)

    # Create the action client node
    action_client = NavigateToGoalClient()

    # Validate and unpack the goal
    try:
        goal_x, goal_y = action_client.goal  # Attempt unpacking
        if not (isinstance(goal_x, (int, float)) and isinstance(goal_y, (int, float))):
            raise ValueError("Goal coordinates must be numeric.")
    except (TypeError, ValueError) as e:
        action_client.get_logger().error(
            f"Invalid goal format: {action_client.goal}. Expected a tuple or list with two numeric elements. Error: {e}"
        )
        return

    # Send goal and wait until it’s completed
    action_client.send_goal(goal_x, goal_y)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        action_client.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()

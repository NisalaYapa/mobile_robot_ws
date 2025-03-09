#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from smrr_interfaces.action import NavigateToGoal
import argparse
import yaml
import os


class NavigateToGoalClient(Node):
    def __init__(self, goal_x=None, goal_y=None):
        super().__init__('navigate_to_goal_client')

        # Create an action client for 'NavigateToGoal'
        self._action_client = ActionClient(self, NavigateToGoal, 'navigate_to_goal')

        # Store the goal handle for future use (cancellation, etc.)
        self.goal_handle = None

        # Load the YAML config file
        package_path = os.path.dirname(__file__)  # Current file's directory
        config_path = os.path.join(package_path, 'config', 'scenario_config.yaml')

        # Load goal from YAML if not provided via CLI
        try:
            with open(config_path, 'r') as file:
                configs = yaml.safe_load(file)

            node_name = "GoalClient"
            node_configs = configs.get(node_name, {})
            yaml_goal = node_configs.get('goal', (0.0, 0.0))
        except Exception as e:
            self.get_logger().warn(f"Failed to load config file. Error: {e}")
            yaml_goal = (0.0, 0.0)  # Default goal if YAML loading fails

        # Use command-line goal if provided, otherwise use YAML goal
        self.goal = (goal_x, goal_y) if goal_x is not None and goal_y is not None else yaml_goal
        self.get_logger().info(f"Using goal: {self.goal}")

    def send_goal(self, x, y):
        """Send a goal to the action server."""
        goal_msg = NavigateToGoal.Goal()
        goal_msg.goal_x = x
        goal_msg.goal_y = y

        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Send the goal and set up callbacks
        self.get_logger().info(f'Sending goal: x={x}, y={y}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response from the action server."""
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected by server.')
            return

        self.get_logger().info('Goal accepted by server.')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def cancel_goal(self):
        """Cancel the current goal."""
        if self.goal_handle is not None:
            self.get_logger().info("Cancelling goal...")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_callback)
        else:
            self.get_logger().info("No active goal to cancel.")

    def cancel_callback(self, future):
        """Handle goal cancellation response."""
        cancel_response = future.result()
        if cancel_response.accepted:
            self.get_logger().info("Goal cancellation accepted.")
        else:
            self.get_logger().info("Goal cancellation rejected.")

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

        # Reset the goal handle
        self.goal_handle = None


def main(args=None):
    """Main function to run the action client."""
    rclpy.init(args=args)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Send a goal to the NavigateToGoal action server.")
    parser.add_argument("--x", type=float, help="X coordinate of the goal")
    parser.add_argument("--y", type=float, help="Y coordinate of the goal")
    parser.add_argument("--cancel", action='store_true', help="Cancel the current goal")
    parsed_args, _ = parser.parse_known_args()

    # Create the action client with command-line goal arguments
    action_client = NavigateToGoalClient(parsed_args.x, parsed_args.y)

    if parsed_args.cancel:
        action_client.cancel_goal()
    else:
        # Validate the goal format
        try:
            goal_x, goal_y = action_client.goal
            if not (isinstance(goal_x, (int, float)) and isinstance(goal_y, (int, float))):
                raise ValueError("Goal coordinates must be numeric.")
        except (TypeError, ValueError) as e:
            action_client.get_logger().error(f"Invalid goal format: {action_client.goal}. Expected numeric values. Error: {e}")
            return

        # Send the goal
        action_client.send_goal(goal_x, goal_y)

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.cancel_goal()
        action_client.get_logger().info("Keyboard interrupt, shutting down...")
    
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

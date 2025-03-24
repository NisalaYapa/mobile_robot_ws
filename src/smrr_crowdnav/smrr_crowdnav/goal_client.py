#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from smrr_interfaces.action import NavigateToGoal
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped
import yaml
import os
import time
import math
import tf_transformations

class NavigateToGoalClient(Node):
    def __init__(self):
        super().__init__('navigate_to_goal_client')

        # Create action client
        self.create_action_client()

        # Store goal handle and current goal
        self.goal_handle = None
        self.current_goal = None

        # Load default goal from YAML
        package_path = os.path.dirname(__file__)  
        config_path = os.path.join(package_path, 'config', 'scenario_config.yaml')

        try:
            with open(config_path, 'r') as file:
                configs = yaml.safe_load(file)
            node_name = "GoalClient"
            node_configs = configs.get(node_name, {})
            yaml_goal = node_configs.get('goal', [0.0, 0.0])
        except Exception as e:
            self.get_logger().warn(f"Failed to load config file. Error: {e}")
            yaml_goal = [0.0, 0.0]  

        self.default_goal = tuple(yaml_goal)

        # Subscribers for goal and cancel commands
        self.goal_subscriber = self.create_subscription(
            Float32MultiArray, '/goal', self.goal_callback, 10)
        
        self.goal_subscriber = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        self.cancel_subscriber = self.create_subscription(
            Bool, '/cancel_goal', self.cancel_callback, 10)

        self.get_logger().info("NavigateToGoalClient initialized. Listening for goals...")


    def quaternion_to_degrees(self, quaternion):
        """Convert quaternion to yaw angle in degrees."""
        roll, pitch, yaw = tf_transformations.euler_from_quaternion([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        ])
        return math.degrees(yaw)


    def create_action_client(self):
        """(Re)Initialize the action client to allow sending new goals."""
        self._action_client = ActionClient(self, NavigateToGoal, 'navigate_to_goal')

    def goal_callback(self, msg):
        """Handle incoming goals from the topic."""
        if len(msg.data) != 2:
            self.get_logger().error("Invalid goal format! Expected Float32MultiArray with [x, y].")
            return

        new_goal = (msg.data[0], msg.data[1])

        if self.current_goal == new_goal:
            self.get_logger().info(f"Goal {new_goal} is already active.")
            return

        if self.goal_handle is not None:
            self.get_logger().info("New goal received. Cancelling the previous goal...")
            self.cancel_goal(new_goal)
        else:
            self.get_logger().info(f"New goal received: {new_goal}")
            self.send_goal(*new_goal)

    def goal_pose_callback(self, msg):
        """Handle incoming goals from the topic."""

        yaw_degrees = self.quaternion_to_degrees(msg.pose.orientation)
        

        new_goal = (msg.pose.position.x, msg.pose.position.y, yaw_degrees)

        if self.current_goal == new_goal:
            self.get_logger().info(f"Goal {new_goal} is already active.")
            return

        if self.goal_handle is not None:
            self.get_logger().info("New goal received. Cancelling the previous goal...")
            self.cancel_goal(new_goal)
        else:
            self.get_logger().info(f"New goal received: {new_goal}")
            self.send_goal(*new_goal)

    def send_goal(self, x, y,rot):
        """Send a new goal to the action server."""
        self.current_goal = (x, y)
        goal_msg = NavigateToGoal.Goal()
        goal_msg.goal_x = x
        goal_msg.goal_y = y
        goal_msg.goal_rot = rot

        # Ensure action client is available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Send the goal
        self.get_logger().info(f'Sending goal: x={x}, y={y}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle response from the action server."""
        self.goal_handle = future.result()

        if not self.goal_handle or not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected by server.')
            self.current_goal = None
            return

        self.get_logger().info('Goal accepted by server.')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def cancel_goal(self, new_goal=None):
        """Cancel the current goal and optionally send a new one."""
        if self.goal_handle is not None:
            self.get_logger().info("Cancelling goal...")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda future: self.cancel_done_callback(future, new_goal))
        else:
            self.get_logger().info("No active goal to cancel.")
            if new_goal:
                self.get_logger().info(f"Sending new goal: {new_goal}")
                self.send_goal(*new_goal)

    def cancel_done_callback(self, future, new_goal):
        """Handle goal cancellation response."""
        cancel_response = future.result()

        # Correct way to check if the cancellation was accepted
        if cancel_response.return_code == rclpy.action.CancelResponse.ACCEPT:
            self.get_logger().info("Goal successfully cancelled.")
        else:
            self.get_logger().info("Goal cancellation failed.")

        # Reset state
        self.goal_handle = None
        self.current_goal = None

        # Reinitialize action client to avoid stuck state
        self.create_action_client()

        # Introduce a small delay before sending a new goal (if required)
        time.sleep(0.5)

        if new_goal:
            self.get_logger().info(f"Sending new goal after cancellation: {new_goal}")
            self.send_goal(*new_goal)

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

        # Reset state
        self.goal_handle = None
        self.current_goal = None

        # Reinitialize action client to allow new goals
        self.create_action_client()

    def cancel_callback(self, msg):
        """Handle cancel requests from the topic."""
        if msg.data:
            self.cancel_goal()

    

def main(args=None):
    """Main function to run the action client."""
    rclpy.init(args=args)
    node = NavigateToGoalClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel_goal()
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

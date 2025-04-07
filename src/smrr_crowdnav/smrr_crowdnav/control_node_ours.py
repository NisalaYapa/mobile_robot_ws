#!/usr/bin/env python3.8

import rclpy
import tf_transformations
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32MultiArray
import casadi as cs
import numpy as np
from smrr_interfaces.msg import Entities, Footprint, PrefVelocity
from geometry_msgs.msg import TwistStamped, Point, PoseStamped , Twist

from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from time import sleep
from .NewMPCReal_ours import NewMPCReal
from .global_path import DijkstraGlobalPlanner
from .global_path import SimplePathPlanner
from .global_path import AStarPathPlanner
from .include.transform import GeometricTransformations
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.msg import GoalStatus
import asyncio
from smrr_interfaces.action import NavigateToGoal# Custom action file
import yaml
import os
from datetime import datetime
import math
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.stats import norm
from matplotlib.lines import Line2D  # For custom legend creation
from matplotlib.colors import LinearSegmentedColormap  # Op
from matplotlib.animation import FuncAnimation



### This is the main version of control node (action server)
### This has a correctly running action server
### This is taking kalman filters positions and velocities to do the human path prediction 
## Integrated human footprint
### Need to be integrate static obstacles

# Define SelfState class
class SelfState:
    def __init__(self, px, py, vx, vy, theta, omega, gx=0.0, gy=0.0, radius=0.4, v_pref=0.5):
        self.px = px
        self.py = py
        self.vx = vx
        self.vy = vy
        self.theta = theta
        self.omega = omega
        self.gx = gx
        self.gy = gy
        self.radius = radius
        self.v_pref = v_pref
        self.position = (self.px, self.py)
        self.goal_position = (self.gx, self.gy)
        self.velocity = (self.vx, self.vy)

# Define HumanState class
class HumanState:
    def __init__(self, px, py, vx, vy, gx, gy, radius=0.4, v_pref=2):
        self.px = px
        self.py = py
        self.vx = vx
        self.vy = vy
        self.gx = gx
        self.gy = gy
        self.radius = radius
        self.v_pref = v_pref
        self.position = (self.px, self.py)
        self.goal_position = (self.gx, self.gy)
        self.velocity = (self.vx, self.vy)

# Define EnvState class
class EnvState:
    def __init__(self, self_state, human_states=[], static_obs=[]):
        self.self_state = self_state
        self.human_states = human_states
        self.static_obs = static_obs

# ROS 2 Node class with Action Server
class CrowdNavMPCNode(Node):
    def __init__(self):
        super().__init__('crowdnav_mpc_node')

        # Load the YAML config file
        package_path = os.path.dirname(__file__)  # Current file's directory
        config_path = os.path.join(package_path,'config', 'config.yaml')
        
        with open(config_path, 'r') as file:
            configs = yaml.safe_load(file)
        
        # Get parameters for this class
        node_name = "ControlNode"  # Define your node's name
        node_configs = configs.get(node_name, {})

        # Set class attributes for each parameter
        for key, value in node_configs.items():
            setattr(self, key, value)  # Dynamically add attributes

        # Log the loaded parameters
        int_goals = getattr(self, 'Intermediate_Goals', 0)  # Default to 1 if not defined
        self.get_logger().info(f"Loaded Intermediate_goals size: {int_goals}")

        self.int_goals = int_goals
        

        # Initialize MPC
        self.policy = NewMPCReal()
        self.self_state = None
        self.human_states = []
        self.static_obs = []
        self.ready = True
        self.rot_rate = self.create_rate(2) 
        self.starttime = 0
        self.endtime = 0
        self.frozen_steps = 0
        self.social_distances = []
        self.self_path = []
        self.human_paths = []
        self.trajectories = []


        self.self_state = SelfState(px=0.0, py=0.0, vx=0.0, vy=0.0, theta=0.0, omega=0.0)

        # Create subscribers for custom messages
        self.create_subscription(Entities, '/goal_predictor/pos', self.human_position_callback, 10)
        self.create_subscription(Entities, '/goal_predictor/vel', self.human_velocity_callback, 10)
        self.create_subscription(Entities, '/goal_predictor/goals', self.human_goal_callback, 10)
        self.create_subscription(PrefVelocity, '/preffered_velocity_prediction/preferred_velocity', self.human_prefvel_callback, 10)
        #self.create_subscription(Entities, '/local_lines_array', self.static_obs_callback, 10)
        self.create_subscription(Entities, '/local_points', self.static_obs_callback, 10)
        

        self.create_subscription(Footprint, '/object_tracker/footprint_array', self.human_footprint_callback, 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.robot_velocity_callback, 10)

        # Publisher for control commands (v, omega)
        self.action_publisher = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.prediction_publisher = self.create_publisher(MarkerArray, '/smrr_crowdnav/prediction_states_marker', 10)
        self.human_prediction_publisher = self.create_publisher(MarkerArray, '/smrr_crowdnav/human_trajectories', 10)
        self.global_path_publisher = self.create_publisher(MarkerArray, '/smrr_crowdnav/global_path', 10)
        self.get_logger().info("Node initiated")

        self.global_path = []
        
        self.intermediate_goal = -1
        self.final_gx = 0
        self.final_gy = 0



        #self.timer = self.create_timer(0.7, self.publish_commands)
        self.transform = GeometricTransformations(self)

        # Create an instance of ReentrantCallbackGroup
        self.callback_group = ReentrantCallbackGroup()

        # Create Action Server for navigation goals
        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            'navigate_to_goal',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=self.callback_group 
        )

    def goal_callback(self, goal_request):
        self.get_logger().info('Received navigation goal request')        
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel goal')
        control = TwistStamped()
        control.header.stamp = self.get_clock().now().to_msg()
        control.twist.linear.x = 0.0
        control.twist.angular.z = 0.0
        self.action_publisher.publish(control)
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self.get_logger().info('Goal accepted, executing...')

        # Set robot's goal based on action input
        self.final_gx = goal_handle.request.goal_x
        self.final_gy = goal_handle.request.goal_y
        self.final_rot = goal_handle.request.goal_rot
        self.final_goal = (self.final_gx, self.final_gy)
        self.intermediate_goal = -1



        #planner = DijkstraGlobalPlanner(self.static_obs, self.int_goals)
        planner = SimplePathPlanner(self.static_obs, self.int_goals)
        #planner = AStarPathPlanner(self.static_obs, self.int_goals)
    
        #self.global_path = planner.find_path((self.self_state.px, self.self_state.py), self.final_goal)
        self.global_path = planner.find_intermediate_goals((self.self_state.px, self.self_state.py), self.final_goal)
        #self.global_path = planner.find_path_with_intermediate_goals((self.self_state.px, self.self_state.py), self.final_goal)


        
        self.get_logger().info(f"Global Path: {self.global_path}") 




        # for i in range(self.int_goals + 2):
        #     self.global_path.append((
        #         self.self_state.px + i * (self.final_gx - self.self_state.px) / (self.int_goals + 1),
        #         self.self_state.py + i * (self.final_gy - self.self_state.py) / (self.int_goals + 1)
        #     ))

        #self.timer = self.create_timer(0.7, self.publish_commands)
        if not hasattr(self, 'timer_initialized') or not self.timer_initialized:
            self.get_logger().info("timer initalized")


            # Formatted output
            self.starttime = datetime.now()
            self.get_logger().warn(f"Current Time = {self.starttime.strftime('%H:%M:%S')}")
            self.frozen_steps = 0
            self.social_distances = []
            self.self_path = []
            self.human_paths = []
            self.trajectories = []



            self.timer = self.create_timer(0.7, self.publish_commands)
            self.timer_initialized = True

        goal_handle.execute()


    async def execute_callback(self, goal_handle):
        #self.get_logger().info('Executing navigation to goal')

        feedback_msg = NavigateToGoal.Feedback()        
        self.finish = False

        # Loop until the goal is reached or canceled
        while rclpy.ok() and self.finish==False:
            #self.publish_commands()
            #self.get_logger().info("Entering while loop iteration")
            dist_to_goal = np.linalg.norm(np.array(self.self_state.position) - np.array(self.final_goal))

            feedback_msg.distance_to_goal = dist_to_goal
            goal_handle.publish_feedback(feedback_msg)
            #self.get_logger().info(f"feedback",feedback_msg)
            print(f"feedback",feedback_msg)


            status = goal_handle.status
            #self.get_logger().info(f"Status {status}")


            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                control = TwistStamped()
                control.header.stamp = self.get_clock().now().to_msg()
                control.twist.linear.x = 0.0
                control.twist.angular.z = 0.0
                self.action_publisher.publish(control)
                self.get_logger().info('Goal canceled')
                self.cleanup_after_goal()
                self.finish = True
                return NavigateToGoal.Result()

            if dist_to_goal < 0.25:
                goal_handle.succeed()

                status = goal_handle.status
                self.get_logger().info(f"Status {status}")
                result = NavigateToGoal.Result()
                result.success = True
                control = TwistStamped()
                control.header.stamp = self.get_clock().now().to_msg()
                control.twist.linear.x = 0.0
                control.twist.angular.z = 0.0
                self.finish = True
                self.endtime = datetime.now()
                self.get_logger().warn(f"Current Time = {self.endtime.strftime('%H:%M:%S')}")
                self.action_publisher.publish(control)
                if hasattr(self, 'timer') and self.timer is not None:
                    self.timer.cancel()
                    self.destroy_timer(self.timer)
                    self.get_logger().error("timer cancled")
                    self.timer = None
                
                self.get_logger().info('Goal reached successfully. Rotating to the correct angle')

                self.rotate_to_goal_angle(self.final_rot)


                self.get_logger().info('Rotated to the correct angle')

                # self.endtime = datetime.now()
                # self.get_logger().warn(f"Current Time = {self.endtime.strftime('%H:%M:%S')}")
                
                self.plot_summary()
                
                
                
                                
                self.cleanup_after_goal()  # Reset states after successful goal completion
                return result
            
        

        # Publish stop command if the goal was not succeeded
        control = TwistStamped()
        control.header.stamp = self.get_clock().now().to_msg()
        control.twist.linear.x = 0.0
        control.twist.angular.z = 0.0
        self.action_publisher.publish(control)
        self.get_logger().info('Goal not succeeded; published stop command')

        # Finalize with failed result
        goal_handle.succeed()
        result = NavigateToGoal.Result()
        result.success = False
        self.cleanup_after_goal()
        return result

    # Cleanup function to reset the timer and state after each goal
    def cleanup_after_goal(self):
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

        if hasattr(self, 'timer_initialized') or self.timer_initialized:
            self.timer_initialized = False
        
        # Reset all state variables related to goals
        self.final_gx = 0.0
        self.final_gy = 0.0
        self.global_path = []
        self.intermediate_goal = -1
        self.finish = False
        self.get_logger().info('Navigation states reset after goal completion')

    def human_position_callback(self, msg):
        #self.get_logger().info('Human Position Callback')
        self.human_states = []
        current_pos = []
        current_pos.append((self.self_state.px, self.self_state.py)) 
        for i in range(msg.count):
            self.human_states.append(HumanState(px=msg.x[i], py=msg.y[i], vx=0.0, vy=0.0, gx=0.0, gy=0.0))
            dx = msg.x[i] - self.self_state.px
            dy = msg.y[i] - self.self_state.py
            distance = math.sqrt(dx**2 + dy**2)  # Euclidean distance
            self.social_distances.append(distance)
            #self.human_paths.append((msg.x[i], msg.y[i]))
            current_pos.append((msg.x[i], msg.y[i]))
        self.trajectories.append(current_pos)

    def human_velocity_callback(self, msg):
        #self.get_logger().info('Human Velocity Callback')
        for i in range(msg.count):
            try:
                self.human_states[i].vx = msg.x[i]
                self.human_states[i].vy = msg.y[i]
                # self.human_states[i].vx = 0.5
                # self.human_states[i].vy = 0.5
            except:
                pass

    def human_goal_callback(self, msg):
        for i in range(msg.count):
            try:
                #self.human_states[i].gx = 0.0
                #self.human_states[i].gy = 0.0
                self.human_states[i].gx = msg.x[i]
                self.human_states[i].gy = msg.y[i]
                
                
            except:
                pass

    def human_footprint_callback(self, msg):
        for i in range(msg.count):
            try:
                self.human_states[i].radius = msg.d[i] / 2
            except:
                pass

    def human_prefvel_callback(self, msg):
        for i in range(len(msg.preferred_velocities)):
            try:
                self.human_states[i].v_pref= msg.preferred_velocities[i]
                #self.get_logger().info(f"#########PrefVel######### {msg.preferred_velocities[i]}")
            except:
                pass

    def static_obs_callback(self, msg):
        static_x = msg.x
        static_y = msg.y


        self.static_obs = []

        try:

            for i in range(msg.count):
                _x = static_x[i]
                _y = static_y[i]


                point = [_x, _y]

                self.static_obs.append(point)
        
        except:
            pass

    def robot_velocity_callback(self, msg):
        #self.get_logger().info('Robot Velocity Callback')
        linear_x = msg.twist.twist.linear.x
        transformation = self.transform.get_transform('map', 'base_link')

        if transformation is None:
            self.ready = False
            return
        else:
            self.ready = True

        quaternion = (transformation.rotation.x, transformation.rotation.y, transformation.rotation.z, transformation.rotation.w)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

        self.self_state.px = transformation.translation.x
        self.self_state.py = transformation.translation.y
        self.self_state.theta = yaw
        self.self_state.vx = linear_x * np.cos(self.self_state.theta)
        self.self_state.vy = linear_x * np.sin(self.self_state.theta)
        self.self_state.position = (self.self_state.px, self.self_state.py)
        self.self_state.omega = msg.twist.twist.angular.z

        #self.self_path.append((self.self_state.px, self.self_state.py))

    def rotate_to_goal_angle(self, goal_yaw_degrees):
        """Rotate the robot to align with the goal orientation after reaching the goal."""
        self.get_logger().info(f"Rotating to goal angle: {goal_yaw_degrees} degrees")

        # Convert degrees to radians
        goal_yaw_radians = math.radians(goal_yaw_degrees)

        # Create a simple control command (zero linear velocity, nonzero angular velocity)
        control = TwistStamped()
        control.header.stamp = self.get_clock().now().to_msg()
        control.twist.linear.x = 0.0
        control.twist.angular.z = 0.3

        tolerance = 0.05  # Allowable error in radians

        while rclpy.ok():
            # Get the current yaw
            current_yaw = self.self_state.theta

            # Calculate the angular difference
            yaw_error = goal_yaw_radians - current_yaw

            # Normalize the error to [-pi, pi]
            yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

            if abs(yaw_error) < tolerance:
                self.get_logger().info("Reached the goal orientation!")
                break

            # Adjust angular velocity based on the error
            control.twist.angular.z = 0.5 * yaw_error  # Proportional control

            # Publish the command
            self.action_publisher.publish(control)
            self.rot_rate.sleep()

        # Stop rotation
        control.twist.angular.z = 0.0
        self.action_publisher.publish(control)

    def publish_commands(self):
        self.get_logger().info("publishing Commands")
        if self.self_state and self.human_states and self.ready:
            #print("global path", self.global_path)

            if self.intermediate_goal == -1 :
                self.intermediate_goal = 0

            dist_to_int_goal = np.linalg.norm(np.array(self.self_state.position) - np.array(self.global_path[self.intermediate_goal]))

            if (dist_to_int_goal <= 1.0) and (self.intermediate_goal != (self.int_goals + 1)):
                self.intermediate_goal = self.intermediate_goal + 1

            self.publish_global_path(self.global_path,self.intermediate_goal)

            self.self_state.gx = self.global_path[self.intermediate_goal][0]
            self.self_state.gy = self.global_path[self.intermediate_goal][1]
            self.self_state.goal_position = (self.self_state.gx, self.self_state.gy)

            env_state = EnvState(self.self_state, self.human_states if self.human_states else [] , self.static_obs)
            
            MPC = self.policy.predict(env_state)      

            action = MPC[0]
            next_states = MPC[1]
            human_next_states = MPC[2]

            

            #print(f"action {action}")
            

            if human_next_states == []:
                human_next_states = [[[]]]


            if action != (0,0):
                control = TwistStamped()
                control.header.stamp = self.get_clock().now().to_msg()

                self.publish_next_states(next_states)

                if human_next_states != [[[]]]:
                    self.publish_human_next_states(human_next_states)
        
                dist_to_goal = np.linalg.norm(np.array(self.self_state.position) - np.array(self.final_goal))

                if dist_to_goal >= 0.25:
                    control.twist.linear.x = float(action[0])
                    control.twist.angular.z = float(action[1])
                    self.get_logger().info(f"control {(float(action[0]), float(action[1]))}")
                    self.action_publisher.publish(control)
                    return
                else:
                    control.twist.linear.x = 0.0
                    control.twist.angular.z = 0.0
                    self.action_publisher.publish(control)
                    self.get_logger().info(f"control {0, 0}")
                    return
                
            else:
                control = TwistStamped()
                control.header.stamp = self.get_clock().now().to_msg()
                self.frozen_steps += 1
                control.twist.linear.x = 0.0
                control.twist.angular.z = 0.0
                self.action_publisher.publish(control)
                self.get_logger().info(f"control {0, 0}")
                return
                

            

    def publish_global_path(self, points, current_goal):
        marker_array = MarkerArray()

        # Create a line strip marker for the global path
        line_strip_marker = Marker()
        line_strip_marker.header.frame_id = "map"
        line_strip_marker.header.stamp = self.get_clock().now().to_msg()
        line_strip_marker.ns = "global_path"
        line_strip_marker.id = 1000
        line_strip_marker.type = Marker.LINE_STRIP
        line_strip_marker.action = Marker.ADD
        line_strip_marker.scale.x = 0.05
        line_strip_marker.color.r = 1.0  
        line_strip_marker.color.b = 1.0 
        line_strip_marker.color.a = 1.0

        # Add points to the line strip marker
        for i, point in enumerate(points):
            marker_point = Point()
            marker_point.x = float(point[0])
            marker_point.y = float(point[1])
            marker_point.z = 0.0
            line_strip_marker.points.append(marker_point)

            # If the current point is the current goal, add a sphere marker
            if i == current_goal:
                goal_marker = Marker()
                goal_marker.header.frame_id = "map"
                goal_marker.header.stamp = self.get_clock().now().to_msg()
                goal_marker.ns = "global_path"
                goal_marker.id = 1000 + i
                goal_marker.type = Marker.SPHERE
                goal_marker.action = Marker.ADD
                goal_marker.pose.position.x = marker_point.x
                goal_marker.pose.position.y = marker_point.y
                goal_marker.pose.position.z = 0.0
                goal_marker.scale.x = 0.2
                goal_marker.scale.y = 0.2
                goal_marker.scale.z = 0.2
                goal_marker.color.r = 0.0  # Blue color for the current goal
                goal_marker.color.g = 0.0
                goal_marker.color.b = 1.0
                goal_marker.color.a = 1.0
                goal_marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
                marker_array.markers.append(goal_marker)
                

        # Append the line strip marker to the marker array
        marker_array.markers.append(line_strip_marker)

        # Publish the marker array
        self.global_path_publisher.publish(marker_array)

    def publish_next_states(self, next_states):
        marker_array = MarkerArray()
        line_strip_marker = Marker()
        line_strip_marker.header.frame_id = "map"
        line_strip_marker.header.stamp = self.get_clock().now().to_msg()
        line_strip_marker.ns = "line_strip"
        line_strip_marker.id = 1000
        line_strip_marker.type = Marker.LINE_STRIP
        line_strip_marker.action = Marker.ADD
        line_strip_marker.scale.x = 0.1
        line_strip_marker.color.r = 1.0
        line_strip_marker.color.a = 1.0

        for state in next_states:
            marker_point = Point()
            marker_point.x = float(state[0])
            marker_point.y = float(state[1])
            
            marker_point.z = 0.0
            line_strip_marker.points.append(marker_point)

        marker_array.markers.append(line_strip_marker)
        self.prediction_publisher.publish(marker_array)

    def publish_human_next_states(self, human_next_states):
        marker_array = MarkerArray()
        
        # Loop through each human trajectory
        for human_id, human in enumerate(human_next_states):
            for time_step, position in enumerate(human):
                # Create a marker for each individual point
                point_marker = Marker()
                point_marker.header.frame_id = "map"
                point_marker.header.stamp = self.get_clock().now().to_msg()
                point_marker.ns = f"human_{human_id}_point_{time_step}"
                point_marker.id = human_id * 1000 + time_step  # Unique ID for each point
                point_marker.type = Marker.SPHERE
                point_marker.action = Marker.ADD
                point_marker.scale.x = self.human_states[human_id].radius # Adjust scale for visibility
                point_marker.scale.y = self.human_states[human_id].radius
                point_marker.scale.z = self.human_states[human_id].radius
                point_marker.color.r = 0.0  # Red color for visibility
                point_marker.color.g = 1.0
                point_marker.color.b = 0.0
                point_marker.color.a = 1.0  # Fully opaque
                point_marker.lifetime = rclpy.time.Duration(seconds=0.5).to_msg()  # Markers persist for 5 seconds

                # Set the position of the marker
                point_marker.pose.position.x = float(position[0])
                point_marker.pose.position.y = float(position[1])
                point_marker.pose.position.z = 0.0

                # Add each point as a separate marker in the MarkerArray
                marker_array.markers.append(point_marker)

        # Publish all points as separate markers
        #print(marker_array)
        self.human_prediction_publisher.publish(marker_array)


    def plot_summary(self):

        current_time = self.endtime.strftime("%Y-%m-%d_%H-%M-%S")

        # Create a dynamic filename
        filename = f"social_dis_pdf_{current_time}.png"

        time_difference = self.endtime - self.starttime  # This gives a timedelta object

        # Extract hours, minutes, and seconds from the time difference
        total_seconds = time_difference.total_seconds()
        hours = int(total_seconds // 3600)
        minutes = int((total_seconds % 3600) // 60)
        seconds = int(total_seconds % 60)

        path_data = self.trajectories
        num_timesteps = len(path_data)

        # Create figure with subplots
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))

        # --- Subplot 1: Social Distance Histogram ---
        ax1.hist(self.social_distances, bins=30, density=True, alpha=0.6, color='b')
        ax1.set_title('Social Distance Distribution')
        ax1.set_xlabel('Distance (m)')
        ax1.set_ylabel('Probability Density')
        ax1.grid(True)

        # --- Subplot 2: Robot and Human Visualization ---
        # Create color gradient for time
        time_cmap = plt.cm.viridis(np.linspace(0, 1, num_timesteps))

        # Plot ROBOT PATH as connected line segments
        robot_x = []
        robot_y = []
        for t in range(num_timesteps):
            if len(path_data[t]) > 0:  # Has robot position
                robot_x.append(path_data[t][0][0])
                robot_y.append(path_data[t][0][1])

        # Plot the actual path the robot traveled
        ax2.plot(robot_x, robot_y, 'b-', linewidth=2, label='Robot Path')

        # Add start/end markers for robot
        if len(robot_x) > 0:
            ax2.scatter(robot_x[0], robot_y[0], 
                        color='green', s=150, zorder=5, label='Robot Start')
            ax2.scatter(robot_x[-1], robot_y[-1], 
                        color='red', s=150, zorder=5, label='Robot End')

        # Plot HUMAN POSITIONS as points (colored by timestep)
        for t in range(num_timesteps):
            if len(path_data[t]) > 1:  # Has humans
                # Get all human positions at this timestep
                humans = path_data[t][1:]
                for human in humans:
                    ax2.scatter(
                        human[0], human[1],
                        color=time_cmap[t],
                        s=50,
                        alpha=0.7,
                        edgecolor='k',
                        label=f'Humans t={t}' if t in [0, num_timesteps-1] else ""
                    )

        # Visualization settings
        max_humans = max(len(timestep)-1 for timestep in path_data) if num_timesteps > 0 else 0
        ax2.set_title(f'Robot Path and Human Positions\n(Max Humans: {max_humans})')
        ax2.set_xlabel('X Position (m)')
        ax2.set_ylabel('Y Position (m)')
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')

        # Create simplified legend
        handles = [
            Line2D([0], [0], color='blue', lw=2, label='Robot Path'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=10, label='Robot Start'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10, label='Robot End'),
            Line2D([0], [0], marker='o', color='k', markerfacecolor='purple', markersize=10, label='Early Humans'),
            Line2D([0], [0], marker='o', color='k', markerfacecolor='yellow', markersize=10, label='Late Humans')
        ]
        ax2.legend(handles=handles, bbox_to_anchor=(1.05, 1), loc='upper left')

        # --- Add Info Text ---
        human_counts = [len(timestep)-1 for timestep in path_data]
        info_text = (f"Navigation Time = {hours}:{minutes}:{seconds}\n"
                    f"Frozen Steps = {self.frozen_steps}\n"
                    f"Minimum Social Distance = {min(self.social_distances):.2f} m\n"
                    f"Timesteps = {num_timesteps}\n"
                    f"Human Count Range: {min(human_counts)}-{max(human_counts)}")

        fig.text(0.5, 0.02, info_text, ha='center', va='top', fontsize=10, 
                bbox={'facecolor': 'white', 'alpha': 0.7, 'pad': 5})

        ros_ws_path = os.path.expanduser("~/mobile_robot_ws/src/smrr_crowdnav/smrr_crowdnav/EvalMatrics")
        plt.tight_layout(rect=[0, 0.1, 1, 0.95])
        plt.savefig(os.path.join(ros_ws_path, filename), dpi=300, bbox_inches='tight')
        plt.close()  


        # Use the original trajectories list directly
        # Use the original trajectories list directly


        # path_data = self.trajectories
        # num_timesteps = len(path_data)

        # # Only proceed if we have valid data
        # if num_timesteps == 0 or any(len(timestep) == 0 for timestep in path_data):
        #     self.get_logger().error("Invalid trajectory data - empty timesteps found")
        #     return

        # # Create figure with subplots
        # fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))

        # # --- Subplot 1: Social Distance Histogram ---
        # ax1.hist(self.social_distances, bins=30, density=True, alpha=0.6, color='b')
        # ax1.set_title('Social Distance Distribution')
        # ax1.set_xlabel('Distance (m)')
        # ax1.set_ylabel('Probability Density')
        # ax1.grid(True)

        # # --- Subplot 2: Robot and Human Visualization ---
        # # Create color gradient for time
        # time_cmap = plt.cm.viridis(np.linspace(0, 1, num_timesteps))

        # # Initialize plot elements for animation
        # robot_line, = ax2.plot([], [], 'b-', linewidth=2, label='Robot Path')
        # robot_dot = ax2.scatter([], [], color='green', s=150, zorder=5)
        # human_dots = ax2.scatter([], [], color='orange', s=50, alpha=0.7, edgecolor='k')

        # # Set plot limits based on all data
        # all_x = []
        # all_y = []
        # for timestep in path_data:
        #     if len(timestep) > 0:
        #         all_x.append(timestep[0][0])
        #         all_y.append(timestep[0][1])
        #     if len(timestep) > 1:
        #         for human in timestep[1:]:
        #             all_x.append(human[0])
        #             all_y.append(human[1])

        # padding = 1.0  # meters
        # ax2.set_xlim(min(all_x)-padding, max(all_x)+padding)
        # ax2.set_ylim(min(all_y)-padding, max(all_y)+padding)
        # ax2.set_xlabel('X Position (m)')
        # ax2.set_ylabel('Y Position (m)')
        # ax2.grid(True, alpha=0.3)
        # ax2.axis('equal')

        # # Add start/end markers
        # ax2.scatter(path_data[0][0][0], path_data[0][0][1], 
        #             color='green', s=150, zorder=5, label='Robot Start')
        # ax2.scatter(path_data[-1][0][0], path_data[-1][0][1], 
        #             color='red', s=150, zorder=5, label='Robot End')

        # # Create simplified legend
        # handles = [
        #     Line2D([0], [0], color='blue', lw=2, label='Robot Path'),
        #     Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=10, label='Robot Start'),
        #     Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10, label='Robot End'),
        #     Line2D([0], [0], marker='o', color='orange', markersize=10, label='Human Positions')
        # ]
        # ax2.legend(handles=handles, bbox_to_anchor=(1.05, 1), loc='upper left')

        # # --- Add Info Text ---
        # human_counts = [len(timestep)-1 for timestep in path_data]
        # max_humans = max(human_counts) if human_counts else 0
        # info_text = (f"Navigation Time = {hours}:{minutes}:{seconds}\n"
        #             f"Frozen Steps = {self.frozen_steps}\n"
        #             f"Minimum Social Distance = {min(self.social_distances):.2f} m\n"
        #             f"Timesteps = {num_timesteps}\n"
        #             f"Max Humans: {max_humans}")

        # fig.text(0.5, 0.02, info_text, ha='center', va='top', fontsize=10, 
        #         bbox={'facecolor': 'white', 'alpha': 0.7, 'pad': 5})

        # # Save static plot
        # ros_ws_path = os.path.expanduser("~/mobile_robot_ws/src/smrr_crowdnav/smrr_crowdnav/EvalMatrics")
        # plt.tight_layout(rect=[0, 0.1, 1, 0.95])
        # plt.savefig(os.path.join(ros_ws_path, filename), dpi=300, bbox_inches='tight')

        # # --- Create Animation ---
        # def init():
        #     robot_line.set_data([], [])
        #     robot_dot.set_offsets(np.empty((0, 2)))  # Empty 2D array
        #     human_dots.set_offsets(np.empty((0, 2)))  # Empty 2D array
        #     return robot_line, robot_dot, human_dots

        # def update(frame):
        #     # Update robot path (all points up to current frame)
        #     robot_x = [path_data[t][0][0] for t in range(frame+1)]
        #     robot_y = [path_data[t][0][1] for t in range(frame+1)]
        #     robot_line.set_data(robot_x, robot_y)
            
        #     # Update current robot position
        #     robot_dot.set_offsets([[path_data[frame][0][0], path_data[frame][0][1]]])
            
        #     # Update human positions
        #     if len(path_data[frame]) > 1:
        #         humans = np.array([[human[0], human[1]] for human in path_data[frame][1:]])
        #         human_dots.set_offsets(humans)
        #     else:
        #         human_dots.set_offsets(np.empty((0, 2)))
            
        #     return robot_line, robot_dot, human_dots

        # # Create animation only if we have enough frames
        # if num_timesteps > 1:
        #     ani = FuncAnimation(fig, update, frames=num_timesteps,
        #                         init_func=init, blit=True, interval=200)
            
        #     # Save GIF
        #     gif_filename = os.path.splitext(filename)[0] + '.gif'
        #     try:
        #         ani.save(os.path.join(ros_ws_path, gif_filename), 
        #                 writer='pillow', fps=5, dpi=100)
        #         self.get_logger().info(f"Saved animation to {gif_filename}")
        #     except Exception as e:
        #         self.get_logger().error(f"Failed to save animation: {e}")
        # else:
        #     self.get_logger().warn("Not enough timesteps to create animation")

        # plt.close()



        # Your existing data
                # self.get_logger().warn(f"Navigation Time = {hours}:{minutes}:{seconds}")
                # self.get_logger().warn(f"Frozen Steps= {self.frozen_steps}")
                # social_distances_array = np.asarray(self.social_distances).flatten()
                # self.get_logger().info(f"minimum social distance = {min(self.social_distances)}")
                # self.get_logger().info(f"human_positions = {self.human_paths}")

                # # Assuming you have a path_array = [(x1,y1), (x2,y2), ...]
                # path_array = np.array(self.self_path)  # Convert your path data to numpy array
                # human_paths_array = np.array(self.human_paths)
                # trajectories = np.array(self.trajectories)

                # ros_ws_path = os.path.expanduser("~/mobile_robot_ws/src/smrr_crowdnav/smrr_crowdnav/EvalMatrics")

                # # Create a figure with two subplots (1 row, 2 columns)
                # fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

                # # First subplot - Histogram
                # ax1.hist(social_distances_array, bins=30, density=True, alpha=0.6, color='b')
                # ax1.set_title('Social Distance Distribution')
                # ax1.set_xlabel('Distance')
                # ax1.set_ylabel('Probability Density')
                # ax1.grid(True)

                # # Second subplot - Path
                # ax2.plot(path_array[:, 0], path_array[:, 1], 'b-', linewidth=2, label='Robot Path')
                # ax2.scatter(path_array[0, 0], path_array[0, 1], color='green', s=100, label='Start')
                # ax2.scatter(path_array[-1, 0], path_array[-1, 1], color='red', s=100, label='End')
                # ax2.scatter(human_paths_array[:, 0], human_paths_array[:, 1], color='orange', alpha=0.6, s=50, label='Human Positions')
                # ax2.set_title('Robot Navigation Path')
                # ax2.set_xlabel('X Position')
                # ax2.set_ylabel('Y Position')
                # ax2.grid(True)
                # ax2.legend()

                # # Add the info text below both subplots
                # info_text = (f"Navigation Time = {hours}:{minutes}:{seconds}\n"
                #             f"Frozen Steps = {self.frozen_steps}\n"
                #             f"Minimum Social Distance = {min(self.social_distances)}")

                # fig.text(0.5, 0.02, info_text, ha='center', va='top', fontsize=10, 
                #         bbox={'facecolor': 'white', 'alpha': 0.7, 'pad': 5})

                # # Adjust layout
                # plt.tight_layout(rect=[0, 0.1, 1, 0.95])  # Adjust bottom margin for text

                # full_path = os.path.join(ros_ws_path, filename)

                # # Save the figure
                # plt.savefig(full_path, dpi=300, bbox_inches='tight')
                # plt.close()


                # Convert your array to numpy for easier manipulation
                # Convert your array to numpy


                # Convert your array to numpy
                # First, let's examine your actual data structure
                
                #path_data = np.array(self.trajectories, dtype=object) # shape: (timesteps, agents, coordinates)
                # Use the original trajectories list directly
                # Use the original trajectories list directly
    




def main(args=None):
    rclpy.init(args=args)

    mpc_node = CrowdNavMPCNode()
    executor = MultiThreadedExecutor()
    executor.add_node(mpc_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    mpc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
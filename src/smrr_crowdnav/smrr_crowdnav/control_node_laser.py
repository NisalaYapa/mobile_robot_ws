#!/usr/bin/env python3.8

#This is the basic crowd navigation control node. it is considering static obstacles directly from the laser readings. With this run NewMPCReal with lasers. 

import rclpy
import tf_transformations
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32MultiArray
import casadi as cs
import numpy as np
from smrr_interfaces.msg import Entities
from geometry_msgs.msg import TwistStamped, Point, PoseStamped
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from time import sleep
from .NewMPCReal_laser import NewMPCReal
from .include.transform import GeometricTransformations
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.msg import GoalStatus
from smrr_interfaces.action import NavigateToGoal# Custom action file


### This is an action server of control node
### Thsi is a test node which has static points in (lidar readings) to test the performance

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
    def __init__(self, px, py, vx, vy, gx, gy, radius=0.8, v_pref=1):
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

        # Initialize MPC
        self.policy = NewMPCReal()
        self.self_state = None
        self.human_states = []
        self.ready = True

        self.self_state = SelfState(px=0.0, py=0.0, vx=0.0, vy=0.0, theta=0.0, omega=0.0)

        # Create subscribers for custom messages
        self.create_subscription(Entities, '/smrr_crowdnav/pos_kf', self.human_position_callback, 10)
        self.create_subscription(Entities, '/smrr_crowdnav/vel_kf', self.human_velocity_callback, 10)
        self.create_subscription(Entities, '/goal_predictor/goals', self.human_goal_callback, 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.robot_velocity_callback, 10)

        # Publisher for control commands (v, omega)
        self.action_publisher = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.prediction_publisher = self.create_publisher(MarkerArray, '/smrr_crowdnav/prediction_states_marker', 10)
        self.human_prediction_publisher = self.create_publisher(MarkerArray, '/smrr_crowdnav/human_trajectories', 10)
        self.laser_point_publisher = self.create_publisher(MarkerArray, 'laser_points', 10)


        self.get_logger().info("Node initiated")

        self.laser_data = []

        self.laser_data =[
                (-4.97, -5.12), (-4.23, -5.03), (3.97, -5.08), (1.28, -5.11),
                (-3.95, -5.14), (0.67, -5.07), (-0.34, -5.00), (4.89, -5.15),
                (-2.53, -5.02), (2.12, -5.01), (5.08, -3.87), (5.02, -1.24),
                (5.10, 2.13), (5.04, 4.88), (5.01, -2.51), (5.14, 1.34),
                (5.00, -4.21), (5.07, 3.75), (5.12, -0.12), (5.09, 0.97),
                (3.87, 5.08), (1.23, 5.00), (-2.45, 5.12), (-4.91, 5.01),
                (2.76, 5.09), (-1.12, 5.03), (0.34, 5.04), (-3.88, 5.11),
                (4.12, 5.05), (-0.98, 5.02), (-5.01, 2.34), (-5.14, -1.12),
                (-5.07, -3.75), (-5.11, 1.01), (-5.02, -2.15), (-5.13, 0.88),
                (-5.06, 4.12), (-5.04, -4.88), (-5.10, 3.45), (-5.03, -0.98),
                (-4.98, -5.09), (-3.21, -5.01), (4.00, -5.07), (2.24, -5.05),
                (-2.92, -5.02), (0.87, -5.14), (3.76, -5.11), (-1.34, -5.04),
                (5.06, 1.54), (5.09, -2.89), (5.13, 4.03), (5.05, 3.21),
                (5.01, 0.97), (5.08, -4.12), (5.10, 2.65), (5.03, -0.45),
                (2.12, 5.01), (-4.32, 5.09), (0.98, 5.05), (-2.01, 5.08),
                (4.65, 5.03), (-3.45, 5.11), (1.22, 5.07), (-0.76, 5.04),
                (-5.08, -3.54), (-5.11, 2.98), (-5.06, -1.23), (-5.14, 1.34),
                (-5.02, -0.45), (-5.10, 3.02), (-5.04, -4.32), (-5.13, 4.23),
                (-5.07, 0.89), (-4.23, -5.12), (3.88, -5.09), (1.76, -5.03), 
                (-3.92, -5.11), (0.67, -5.08), (-0.45, -5.10), (4.78, -5.05),
                (-2.89, -5.01), (2.03, -5.06), (5.14, 1.12), (5.07, -0.89),
                (5.02, 3.98), (5.12, -2.32), (5.05, 4.01), (5.09, -3.45),
                (1.98, 5.07), (-4.76, 5.02), (3.21, 5.09), (-1.23, 5.01),
                (4.09, 5.03), (-2.98, 5.05), (0.45, 5.08), (-3.21, 5.12)
            ]


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
            callback_group=self.callback_group  # Pass the instance, not the class
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
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing navigation to goal')

        # Set robot's goal based on action input
        self.self_state.gx = goal_handle.request.goal_x
        self.self_state.gy = goal_handle.request.goal_y
        self.self_state.goal_position = (self.self_state.gx, self.self_state.gy)
        print("goal", self.self_state.gx, self.self_state.gy)
        print("position", self.self_state.px, self.self_state.py)

        feedback_msg = NavigateToGoal.Feedback()

        # Initialize the timer once when the goal is accepted
        if not hasattr(self, 'timer_initialized') or not self.timer_initialized:
            self.timer = self.create_timer(0.7, self.publish_commands)
            self.timer_initialized = True

        # Loop until the goal is reached or canceled
        while rclpy.ok():
            dist_to_goal = np.linalg.norm(np.array(self.self_state.position) - np.array(self.self_state.goal_position))

            feedback_msg.distance_to_goal = dist_to_goal
            goal_handle.publish_feedback(feedback_msg)
            print(f"feedback",feedback_msg)


            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                control = TwistStamped()
                control.header.stamp = self.get_clock().now().to_msg()
                control.twist.linear.x = 0.0
                control.twist.angular.z = 0.0
                self.action_publisher.publish(control)

                self.get_logger().info('Goal canceled')
                self.cleanup_after_goal()
                return NavigateToGoal.Result()

            if dist_to_goal < 0.25:
                goal_handle.succeed()
                result = NavigateToGoal.Result()
                result.success = True

                control = TwistStamped()
                control.header.stamp = self.get_clock().now().to_msg()
                control.twist.linear.x = 0.0
                control.twist.angular.z = 0.0
                self.action_publisher.publish(control)

                self.get_logger().info('Goal reached successfully')
                self.cleanup_after_goal()
                return result

            rclpy.spin_once(self)

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
        if hasattr(self, 'timer'):
            self.timer.cancel()
            self.timer_initialized = False
            self.get_logger().info('Resetting timer and state after goal completion')




    def human_position_callback(self, msg):
        self.human_states = []
        for i in range(msg.count):
            self.human_states.append(HumanState(px=msg.x[i], py=msg.y[i], vx=0.0, vy=0.0, gx=0.0, gy=0.0))

    def human_velocity_callback(self, msg):
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
                self.human_states[i].gx = msg.x[i]
                self.human_states[i].gy = msg.y[i]
            except:
                pass

    def robot_velocity_callback(self, msg):
        linear_x = msg.twist.twist.linear.x
        transformation = self.transform.get_transform('map', 'base_link')

        if transformation is None:
            self.ready = False
            #self.get_logger().info("Robot not ready: No valid transformation")
            return
        else:
            self.ready = True
            #self.get_logger().info("Robot ready: Transformation found")

        quaternion = (transformation.rotation.x, transformation.rotation.y, transformation.rotation.z, transformation.rotation.w)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

        self.self_state.px = transformation.translation.x
        self.self_state.py = transformation.translation.y
        self.self_state.theta = yaw
        self.self_state.vx = linear_x * np.cos(self.self_state.theta)
        self.self_state.vy = linear_x * np.sin(self.self_state.theta)
        self.self_state.position = (self.self_state.px, self.self_state.py)
        self.self_state.omega = msg.twist.twist.angular.z

    def publish_commands(self):
        if self.self_state and self.human_states and self.ready:
            env_state = EnvState(self.self_state, self.human_states if self.human_states else [])
            MPC = self.policy.predict(env_state,self.laser_data)

            action = MPC[0]
            next_states = MPC[1]
            if MPC != (0,0):
                human_next_states = MPC[2]
            else:
                human_next_states = [[[]]]

            if action != 0:
                control = TwistStamped()
                control.header.stamp = self.get_clock().now().to_msg()
                self.publish_next_states(next_states)
                self.publish_human_next_states(human_next_states)
                self.publish_laser(self.laser_data)

                dist_to_goal = np.linalg.norm(np.array(self.self_state.position) - np.array(self.self_state.goal_position))

                if dist_to_goal >= 0.25:
                    control.twist.linear.x = float(action[0])
                    control.twist.angular.z = float(action[1])
                    self.action_publisher.publish(control)
                else:
                    control.twist.linear.x = 0.0
                    control.twist.angular.z = 0.0
                    self.action_publisher.publish(control)

    def publish_next_states(self, next_states):
        marker_array = MarkerArray()
        line_strip_marker = Marker()
        line_strip_marker.header.frame_id = "map"
        line_strip_marker.header.stamp = self.get_clock().now().to_msg()
        line_strip_marker.ns = "line_strip"
        line_strip_marker.id = 1000
        line_strip_marker.type = Marker.LINE_STRIP
        line_strip_marker.action = Marker.ADD
        line_strip_marker.scale.x = 0.05
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


    # def publish_human_next_states(self, human_next_states):
    #     print("publishing markers", human_next_states)
    #     marker_array = MarkerArray()
        
    #     for human_id, human in enumerate(human_next_states):
    #         line_strip_marker = Marker()
    #         line_strip_marker.header.frame_id = "map"
    #         line_strip_marker.header.stamp = self.get_clock().now().to_msg()
    #         line_strip_marker.ns = f"human_trajectory_{human_id}"
    #         line_strip_marker.id = human_id
    #         line_strip_marker.type = Marker.LINE_STRIP
    #         line_strip_marker.action = Marker.ADD
    #         line_strip_marker.scale.x = 0.5  # Adjust as needed
    #         line_strip_marker.color.r = 0.0
    #         line_strip_marker.color.g = 1.0
    #         line_strip_marker.color.b = 0.0
    #         line_strip_marker.color.a = 1.0
    #         line_strip_marker.lifetime = rclpy.time.Duration(seconds=5).to_msg()  # Markers persist for 5 seconds

    #         for time_step in human:
    #             marker_point = Point()
    #             marker_point.x = float(time_step[0]) if len(time_step) > 0 else 0.0
    #             marker_point.y = float(time_step[1]) if len(time_step) > 1 else 0.0
    #             marker_point.z = 0.0
    #             line_strip_marker.points.append(marker_point)
            
    #         marker_array.markers.append(line_strip_marker)

    #     print(marker_array)
    #     self.human_prediction_publisher.publish(marker_array)

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
                point_marker.scale.x = 0.2  # Adjust scale for visibility
                point_marker.scale.y = 0.2
                point_marker.scale.z = 0.2
                point_marker.color.r = 1.0  # Red color for visibility
                point_marker.color.g = 0.0
                point_marker.color.b = 0.0
                point_marker.color.a = 1.0  # Fully opaque
                point_marker.lifetime = rclpy.time.Duration(seconds=5).to_msg()  # Markers persist for 5 seconds

                # Set the position of the marker
                point_marker.pose.position.x = float(position[0])
                point_marker.pose.position.y = float(position[1])
                point_marker.pose.position.z = 0.0

                # Add each point as a separate marker in the MarkerArray
                marker_array.markers.append(point_marker)

        # Publish all points as separate markers
        #print(marker_array)
        self.human_prediction_publisher.publish(marker_array)


    def publish_laser(self, laser_data):        
        marker_array = MarkerArray()
        points_marker = Marker()
        points_marker.header.frame_id = "map"
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.ns = "laser_points"
        points_marker.id = 2000
        points_marker.type = Marker.POINTS  # Change to POINTS
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.1  # Point size in x (width)
        points_marker.scale.y = 0.1  # Point size in y (height)
        points_marker.color.r = 1.0  # Red color
        points_marker.color.g = 0.0
        points_marker.color.b = 0.0
        points_marker.color.a = 1.0  # Fully opaque

        for point in laser_data:
            marker_point = Point()
            marker_point.x = float(point[0])
            marker_point.y = float(point[1])
            marker_point.z = 0.0  # Assuming 2D points at z = 0
            points_marker.points.append(marker_point)

        marker_array.markers.append(points_marker)
        self.laser_point_publisher.publish(marker_array)






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

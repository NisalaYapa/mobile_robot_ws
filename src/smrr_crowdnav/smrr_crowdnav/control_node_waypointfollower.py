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
from smrr_interfaces.msg import Entities
from geometry_msgs.msg import TwistStamped, Point, PoseStamped
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from time import sleep
from .NewMPCReal_waypointfollower import NewMPCReal
from .include.transform import GeometricTransformations
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.msg import GoalStatus
from smrr_interfaces.action import NavigateToGoal# Custom action file


### This is an action server of control node
### This is dividing global path onto several waypoints that can be tracked instead of final goal.

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
        self.create_subscription(Entities, '/laser_data_array', self.human_position_callback, 10)
        self.create_subscription(Entities, '/vel', self.human_velocity_callback, 10)
        self.create_subscription(Entities, '/goals', self.human_goal_callback, 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.robot_velocity_callback, 10)

        # Publisher for control commands (v, omega)
        self.action_publisher = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.prediction_publisher = self.create_publisher(MarkerArray, 'prediction_states_marker', 10)
        self.human_prediction_publisher = self.create_publisher(MarkerArray, 'human_trajectories', 10)
        self.global_path_publisher = self.create_publisher(MarkerArray, 'global_path', 10)
        self.get_logger().info("Node initiated")

        self.global_path = []
        self.num_int_goals = 3
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
        self.final_gx= goal_handle.request.goal_x
        self.final_gy= goal_handle.request.goal_y
        self.final_goal = (self.final_gx, self.final_gy)


        for i in range(self.num_int_goals + 2):
            self.global_path.append((self.self_state.px + i*(self.final_gx - self.self_state.px)/(self.num_int_goals + 1),self.self_state.py + i*(self.final_gy - self.self_state.py)/(self.num_int_goals + 1) ))

        print("global path", self.global_path)
        print("goal", self.final_gx, self.final_gy)
        print("position", self.self_state.px, self.self_state.py)

        feedback_msg = NavigateToGoal.Feedback()

        # Initialize the timer once when the goal is accepted
        if not hasattr(self, 'timer_initialized') or not self.timer_initialized:
            self.timer = self.create_timer(0.7, self.publish_commands)
            self.timer_initialized = True

        # Loop until the goal is reached or canceled
        while rclpy.ok():
            dist_to_goal = np.linalg.norm(np.array(self.self_state.position) - np.array(self.final_goal))

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

    def publish_commands(self):
        if self.self_state and self.human_states and self.ready:
            print("global path", self.global_path)

            if self.intermediate_goal == -1 :
                self.intermediate_goal = 0

            dist_to_int_goal = np.linalg.norm(np.array(self.self_state.position) - np.array(self.global_path[self.intermediate_goal]))

            if (dist_to_int_goal <= 1.0) and (self.intermediate_goal != (self.num_int_goals + 1)):
                self.intermediate_goal = self.intermediate_goal + 1

            print("intermediate goal", self.intermediate_goal)

            print("distance_to_int_goal", dist_to_int_goal)

            self.publish_global_path(self.global_path,self.intermediate_goal)

            self.self_state.gx = self.global_path[self.intermediate_goal][0]
            self.self_state.gy = self.global_path[self.intermediate_goal][1]
            self.self_state.goal_position = (self.self_state.gx, self.self_state.gy)

            env_state = EnvState(self.self_state, self.human_states if self.human_states else [])
            MPC = self.policy.predict(env_state)

            



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
        
                dist_to_goal = np.linalg.norm(np.array(self.self_state.position) - np.array(self.final_goal))

                if dist_to_goal >= 0.25:
                    control.twist.linear.x = float(action[0])
                    control.twist.angular.z = float(action[1])
                    self.action_publisher.publish(control)
                else:
                    control.twist.linear.x = 0.0
                    control.twist.angular.z = 0.0
                    self.action_publisher.publish(control)

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
        line_strip_marker.color.r = 1.0  # Red color for the path
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

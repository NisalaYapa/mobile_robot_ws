#!/usr/bin/env python3.8

import rclpy
import tf_transformations
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import casadi as cs
import numpy as np
from geometry_msgs.msg import TwistStamped, Point
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
from smrr_interfaces.msg import Entities
from .NewMPCReal_kf import NewMPCReal
from .include.transform import GeometricTransformations

### This is a simplified code of controlnode in publisher-subscriber achitechture. 
### It is only running the NewMPC_kf (In here it is not computing the MPC but it is only predicting human paths using ORCA all)
### This can visualize predicted human paths when human state is using Kalman Filter vs not using Kalman Filter


# Define SelfState class
class SelfState:
    def __init__(self, px, py, vx, vy, theta, omega, gx=5, gy=-5, radius=0.2, v_pref=0.5):
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
    def __init__(self, px, py, vx, vy, gx, gy, radius=0.15, v_pref=1):
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
    def __init__(self, self_state, human_states, static_obs=[]):
        self.self_state = self_state
        self.human_states = human_states
        self.static_obs = static_obs

# ROS 2 Node class
class KalmanTest(Node):
    def __init__(self):
        super().__init__('kalman_vs_no_kalman')

        self.policy = NewMPCReal()


        # Initialize state variables
        self.self_state = None
        self.human_states_kf = []
        self.human_states = []
        self.ready = True
        
        self.self_state = SelfState(px=0.0, py=0.0, vx=0.0, vy=0.0, theta=0.0, omega=0.0)

        # Create subscribers for the custom messages
        # Create subscribers for the custom messages
        self.create_subscription(Entities, '/goal_predictor/pos', self.human_position_callback, 10)
        self.create_subscription(Entities, '/goal_predictor/vel', self.human_velocity_callback, 10)
        self.create_subscription(Entities, '/smrr_crowdnav/pos_kf', self.human_position_kf_callback, 10)
        self.create_subscription(Entities, '/smrr_crowdnav/vel_kf', self.human_velocity_kf_callback, 10)

        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.robot_velocity_callback, 10)

        self.human_prediction_publisher_no_kf = self.create_publisher(MarkerArray, '/smrr_crowdnav/human_trajectories_no_kf', 10)
        self.human_prediction_publisher_kf = self.create_publisher(MarkerArray, '/smrr_crowdnav/human_trajectories_kf', 10)
        
        
        # Publisher to send control commands (v, omega)
        #self.prediction_publisher = self.create_publisher(MarkerArray, 'prediction_states_marker', 10)
       
        self.get_logger().info("Node initiated")

        self.timer = self.create_timer(0.3, self.publish_commands)
        self.timer = self.create_timer(0.3, self.publish_commands_kf)

        self.transform = GeometricTransformations(self)                   

    def human_position_callback(self, msg):
        # Update human states with position data
        self.human_states = []
        for i in range(msg.count):
            self.human_states.append(HumanState(px=msg.x[i], py=msg.y[i], vx=0.0, vy=0.0, gx=0.0, gy=0.0))
        #print(len(self.human_states))

    def human_velocity_callback(self, msg):
        # Update human states with velocity data
        for i in range(msg.count):
            try:
                self.human_states[i].vx = msg.x[i]
                self.human_states[i].vy = msg.y[i]
            except :
                pass

    def human_position_kf_callback(self, msg):
        # Update human states with position data
        self.human_states_kf = []
        for i in range(msg.count):
            self.human_states_kf.append(HumanState(px=msg.x[i], py=msg.y[i], vx=0.0, vy=0.0, gx=0.0, gy=0.0))
        #print(len(self.human_states))

    def human_velocity_kf_callback(self, msg):
        # Update human states with velocity data
        for i in range(msg.count):
            try:
                self.human_states_kf[i].vx = msg.x[i]
                self.human_states_kf[i].vy = msg.y[i]
            except :
                pass

    def human_goal_callback(self, msg):
        # Update human states with goal data
        for i in range(msg.count):
            try:
                self.human_states[i].gx = msg.x[i]
                self.human_states[i].gy = msg.y[i]
            except :
                pass

    def human_goal_kf_callback(self, msg):
        # Update human states with goal data
        for i in range(msg.count):
            try:
                self.human_states_kf[i].gx = msg.x[i]
                self.human_states_kf[i].gy = msg.y[i]
            except :
                pass

    def robot_velocity_callback(self, msg):
            
            linear_x = msg.twist.twist.linear.x
            #print(f"linear x{linear_x}")

            transformation   = self.transform.get_transform('map', 'base_link') 

            if transformation is None:
                self.ready = False
                self.get_logger().info("Robot not ready: No valid transformation")
                return
            else:
                self.ready = True
                #self.get_logger().info("Robot ready: Transformation found")

            quaternion       = (transformation.rotation.x, transformation.rotation.y, transformation.rotation.z, transformation.rotation.w)
            roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

            
            self.self_state.px    = transformation.translation.x
            self.self_state.py    = transformation.translation.y
            self.self_state.theta = yaw
            self.self_state.vx    = linear_x*np.cos(self.self_state.theta)
            self.self_state.vy    = linear_x*np.sin(self.self_state.theta)
            self.self_state.position = (self.self_state.px, self.self_state.py)
            self.self_state.omega = msg.twist.twist.angular.z
       
            

    def publish_commands(self):
        # Predict and publish control commands
        if self.self_state and self.human_states and self.ready:
            env_state = EnvState(self.self_state, self.human_states if self.human_states else [])
            MPC = self.policy.predict(env_state)      

        

            human_next_states = MPC
            self.publish_human_next_states(human_next_states)


    def publish_commands_kf(self):
        # Predict and publish control commands
        if self.self_state and self.human_states_kf and self.ready:
            env_state = EnvState(self.self_state, self.human_states_kf if self.human_states_kf else [])
            MPC = self.policy.predict(env_state)      


            human_next_states_kf = MPC
            self.publish_human_next_states_kf(human_next_states_kf)


            

    def publish_human_next_states(self, human_next_states):
        marker_array = MarkerArray()
        
        # Loop through each human trajectory
        for human_id, human in enumerate(human_next_states):
            # Create a line strip marker for the trajectory
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = f"human_{human_id}_trajectory"
            line_marker.id = human_id  # Unique ID for each human's trajectory
            line_marker.type = Marker.LINE_STRIP  # Create a line strip
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.02 # Thickness of the line
            line_marker.color.r = 1.0  # Red color for visibility
            line_marker.color.g = 0.0
            line_marker.color.b = 1.0
            line_marker.color.a = 1.0  # Fully opaque
            line_marker.lifetime = rclpy.time.Duration(seconds=5).to_msg()  # Line persists for 5 seconds

            # Add each position in the human trajectory as a point in the line strip
            for position in human:
                point = Point()
                point.x = float(position[0])
                point.y = float(position[1])
                point.z = 0.0
                line_marker.points.append(point)

                # Create a sphere marker for this point
                point_marker = Marker()
                point_marker.header.frame_id = "map"
                point_marker.header.stamp = self.get_clock().now().to_msg()
                point_marker.ns = f"human_{human_id}_points"
                point_marker.id = human_id * 1000 + len(line_marker.points)  # Unique ID for each point
                point_marker.type = Marker.SPHERE
                point_marker.action = Marker.ADD
                point_marker.scale.x = 0.12  # Adjust scale for visibility
                point_marker.scale.y = 0.12
                point_marker.scale.z = 0.12
                point_marker.color.r = 1.0  # Blue color for points
                point_marker.color.g = 0.0
                point_marker.color.b = 0.0
                point_marker.color.a = 1.0  # Fully opaque
                point_marker.pose.position.x = point.x
                point_marker.pose.position.y = point.y
                point_marker.pose.position.z = point.z
                point_marker.lifetime = rclpy.time.Duration(seconds=5).to_msg()

                # Add the point marker to the MarkerArray
                marker_array.markers.append(point_marker)

            # Add the line strip marker to the MarkerArray
            marker_array.markers.append(line_marker)

        # Publish all trajectories as separate line and point markers
        self.human_prediction_publisher_no_kf.publish(marker_array)

    def publish_human_next_states_kf(self, human_next_states_kf):
        marker_array = MarkerArray()
        
        # Loop through each human trajectory
        for human_id, human in enumerate(human_next_states_kf):
            # Create a line strip marker for the trajectory
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = f"human_{human_id}_trajectory"
            line_marker.id = human_id  # Unique ID for each human's trajectory
            line_marker.type = Marker.LINE_STRIP  # Create a line strip
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.02 # Thickness of the line
            line_marker.color.r = 0.0  # Red color for visibility
            line_marker.color.g = 1.0
            line_marker.color.b = 1.0
            line_marker.color.a = 1.0  # Fully opaque
            line_marker.lifetime = rclpy.time.Duration(seconds=5).to_msg()  # Line persists for 5 seconds

            # Add each position in the human trajectory as a point in the line strip
            for position in human:
                point = Point()
                point.x = float(position[0])
                point.y = float(position[1])
                point.z = 0.0
                line_marker.points.append(point)

                # Create a sphere marker for this point
                point_marker = Marker()
                point_marker.header.frame_id = "map"
                point_marker.header.stamp = self.get_clock().now().to_msg()
                point_marker.ns = f"human_{human_id}_points"
                point_marker.id = human_id * 1000 + len(line_marker.points)  # Unique ID for each point
                point_marker.type = Marker.SPHERE
                point_marker.action = Marker.ADD
                point_marker.scale.x = 0.12  # Adjust scale for visibility
                point_marker.scale.y = 0.12
                point_marker.scale.z = 0.12
                point_marker.color.r = 0.0  # Blue color for points
                point_marker.color.g = 1.0
                point_marker.color.b = 0.0
                point_marker.color.a = 1.0  # Fully opaque
                point_marker.pose.position.x = point.x
                point_marker.pose.position.y = point.y
                point_marker.pose.position.z = point.z
                point_marker.lifetime = rclpy.time.Duration(seconds=5).to_msg()

                # Add the point marker to the MarkerArray
                marker_array.markers.append(point_marker)

            # Add the line strip marker to the MarkerArray
            marker_array.markers.append(line_marker)

        # Publish all trajectories as separate line and point markers
        self.human_prediction_publisher_kf.publish(marker_array)


    

def main(args=None):
    rclpy.init(args=args)
    kalman_vs_no_kalman = KalmanTest()

    try:
        rclpy.spin(kalman_vs_no_kalman)
    except KeyboardInterrupt:
        pass
    finally:
        kalman_vs_no_kalman.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
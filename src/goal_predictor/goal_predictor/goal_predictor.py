import rclpy
import numpy                as np
import matplotlib.pyplot    as plt
from scipy.stats    import norm
from rclpy.node     import Node
from smrr_interfaces.msg import Entities, Buffer
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import colorsys

class GoalPredictor(Node):
    def __init__(self):
        super().__init__('goal_predictor')


        # subscribe to the buffer topic
        self.pos_subscription = self.create_subscription(Buffer, '/human_data_buffer/buffer', self.predictor_callback, 10)
        
        self.pos_publisher  = self.create_publisher(Entities,'/goal_predictor/pos', 10)
        self.vel_publisher  = self.create_publisher(Entities,'/goal_predictor/vel', 10)
        self.goal_publisher = self.create_publisher(Entities,'/goal_predictor/goals', 10)

        self.human_goals = self.create_publisher(MarkerArray, '/goal_predictor/human_goals_marker', 10)
        self.human_positions = self.create_publisher(MarkerArray, '/goal_predictor/human_positions_marker', 10)
        self.human_velocities = self.create_publisher(MarkerArray, '/goal_predictor/human_velocity_marker', 10)
        self.destination_locations = self.create_publisher(MarkerArray, '/goal_predictor/destinations', 10)


        self.pedestrian_pos = [] 
        self.pedestrian_vel = []
        self.path_buffer    = 5         # Number of past readings kept for each agent
        self.dt             = 0.2       # Position publisher rate
        self.sigma_phi      = 0.1

        self.max_agent_buffer = 12

        self.destinations = np.array(
            [[5.0, 8.0], 
           [2.0, -5.0], 
           [5, 3], 
           [-1, 2], 
           [3, 0], 
           [-1, -3], 
           [8.0, 3], 
           [-5, 5], 
           [9, -2], 
           [6, 0]])


        self.agents = Entities()
        self.vel    = Entities()
        self.goals  = Entities()

    def predictor_callback(self, msg):

        # extract agent positions from buffer
        # 0th index of buffer same as 0th index of x_positions
        xpositions_of_agents = []
        ypositions_of_agents = []
        xvelocities_of_agents = []
        yvelocities_of_agents = []

        for i in range(msg.agent_count):
            xvelocities_of_agents.append(msg.x_velocities[i].float_data[-1]) # most recent data in the buffer
            yvelocities_of_agents.append(msg.y_velocities[i].float_data[-1]) # most recent data in the buffer
            xpositions_of_agents.append(msg.x_positions[i].float_data[-1]) # most recent data in the buffer
            ypositions_of_agents.append(msg.y_positions[i].float_data[-1]) # most recent data in the buffer

        # extract agent count from buffer 
        self.agents.count    = msg.agent_count
        self.agents.x        = xpositions_of_agents
        self.agents.y        = ypositions_of_agents  
        # extract x and y mean velocities for each agents
        self.vel.count       = self.agents.count
        self.vel.x           = xvelocities_of_agents 
        self.vel.y           = yvelocities_of_agents

        
        
        self.goals.count     = self.agents.count
        self.goals.x         = [0.0]*self.vel.count  
        self.goals.y         = [0.0]*self.vel.count 

        if(self.agents.count!=0): # Handling errors in human detection
            self.update_path()
            self.predict_goals()
            self.vel_publisher.publish(self.vel)
            self.goal_publisher.publish(self.goals)
            self.pos_publisher.publish(self.agents)

    def update_path(self):
        if (len(self.pedestrian_pos) == 0):
            self.pedestrian_pos = np.zeros([self.max_agent_buffer, self.path_buffer*2], dtype='float') # Creating buffer for each agent's position
            self.pedestrian_vel = np.zeros_like(self.pedestrian_pos)                             # Creating buffer for each agent's velocity
        else:
            for i in range(self.agents.count):
                self.pedestrian_pos[i][:2*self.path_buffer-2] = self.pedestrian_pos[i][2:2*self.path_buffer] # Shift the buffer values to left (2 positions)
                self.pedestrian_pos[i][2*self.path_buffer-2:] = self.agents.x[i],self.agents.y[i]                    # Include latest positions as the last element of the buffer
                
            for j in range(self.agents.count):
                self.pedestrian_vel[j][:2*self.path_buffer-2] = self.pedestrian_vel[j][2:2*self.path_buffer] # Shifting velocity buffer
                
                # commented
                #self.vel.x[j] = ((self.pedestrian_pos[j][-2] + self.pedestrian_pos[j][-4]) - (self.pedestrian_pos[j][-6] + self.pedestrian_pos[j][-8]))/self.dt
                #self.vel.y[j] = ((self.pedestrian_pos[j][-1] + self.pedestrian_pos[j][-3]) - (self.pedestrian_pos[j][-5] + self.pedestrian_pos[j][-7]))/self.dt  # Velocity calculation
                
                self.pedestrian_vel[j][2*self.path_buffer-2:] = [self.vel.x[j], self.vel.y[j]]

    def pedestrian_state(self, timeStep , pd = 0): # Set pedestrian to 0 for simulation purposes
        # Reading pedestrian state from pos, vel buffers
        pos = tuple(self.pedestrian_pos[pd][2*self.path_buffer-2*timeStep-2:2*self.path_buffer-2*timeStep])
        vel = tuple(self.pedestrian_vel[pd][2*self.path_buffer-2*timeStep-2:2*self.path_buffer-2*timeStep])
        
        return pos, vel

    def compute_angle(self, pos, vel, dest):
        direction_to_dest = dest - pos # Angle to goal
        norm_vel = np.linalg.norm(vel)
        norm_dir = np.linalg.norm(direction_to_dest)
        
        # Check if any of the vectors have zero magnitude
        if norm_vel == 0 or norm_dir == 0:
            return 0
        
        # Clamp the value to avoid invalid input for arccos
        cos_theta = np.dot(vel, direction_to_dest) / (norm_vel * norm_dir) # Calculate angle between goal direction and moving direction
        cos_theta = np.clip(cos_theta, -1.0, 1.0)  

        angle = np.arccos(cos_theta)
        return angle

    # Compute the probability P(sp(t+Δt)|Dm)
    def compute_probability(self, pos, vel, dest, sigma):
        angle   = self.compute_angle(pos, vel, dest)
        prob    = norm.pdf(angle, 0, sigma)  # Gaussian distribution
        return prob

    # Bayesian classifier for destination prediction
    def predict_destination(self, D, w=5):
        # Store recent pedestrian states
        for k in range(self.agents.count):
            recent_states = []
            for i in range(w):
                pos, vel = self.pedestrian_state(i , pd= k)
                recent_states.append((pos, vel))
            
            destination_probs = []
            for dest in D:
                # Compute joint probability for each destination
                joint_prob = 1
                for pos, vel in recent_states:
                    joint_prob *= self.compute_probability(pos, vel, dest, self.sigma_phi)
                
                destination_probs.append(joint_prob)
            
            # Normalize probabilities
            destination_probs = np.array(destination_probs) / np.sum(destination_probs)

            self.goals.x[k] = D[np.argmax(destination_probs)][0]
            self.goals.y[k] = D[np.argmax(destination_probs)][1]

        self.publish_goal_marker()
        self.publish_position_marker()
        self.publish_velocity_marker()
        self.destinations_marker()

        return self.goals
        #return D[np.argmax(destination_probs)], destination_probs

    def predict_goals(self):
        ######################################### For visualization  ###################################################
        # agent_num = 5 
        # pos, vel = self.pedestrian_state(timeStep=0, pd = agent_num )
        # plt.clf()  
        # plt.quiver(pos[0], pos[1], vel[0], vel[1], color='r', scale=8)  # Pedestrian velocity
        # plt.scatter(self.destinations[:, 0], self.destinations[:, 1], c='blue', label='Destinations' , s= 50)
        
        pred_dest = self.predict_destination(self.destinations)

        ######################################### For visualization  ###################################################

        # plt.scatter(pred_dest.x[agent_num], pred_dest.y[agent_num], c='green', label='Predicted Destination', marker='X' , s= 200)
        # plt.legend()
        # plt.draw()  
        # plt.pause(0.01)  

    def destinations_marker(self):
        marker_array = MarkerArray()  
        count = len(self.destinations)

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



        for i in range(count):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "destinations"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = self.destinations[i][0]# x position
            marker.pose.position.y = self.destinations[i][1]# y position
            marker.pose.position.z = 0.0  # z position (assumed flat plane)
            marker.scale.x = 1.0 # Sphere size in x
            marker.scale.y = 1.0  # Sphere size in y
            marker.scale.z = 0.01 # Sphere size in z
            marker.color.a = 0.2 # Transparency
            color = colors[9]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            # Set lifetime of the marker
            marker.lifetime = Duration(sec=1, nanosec=0)  # Marker lasts for 1 second
            marker_array.markers.append(marker)
        self.destination_locations.publish(marker_array)

    def publish_goal_marker(self):
        marker_array = MarkerArray()  
        count = len(self.goals.x)
        x_pos  = self.goals.x
        y_pos = self.goals.y

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
            marker.ns = "human_goals"
            marker.id = human_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x_pos[human_id]+(human_id/5)# x position
            marker.pose.position.y = y_pos[human_id] + (human_id/5)# y position
            marker.pose.position.z = 0.0  # z position (assumed flat plane)
            marker.scale.x = 0.2  # Sphere size in x
            marker.scale.y = 0.2  # Sphere size in y
            marker.scale.z = 0.1 # Sphere size in z
            marker.color.a = 1.0  # Transparency
            color = colors[human_id % len(colors)]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            # Set lifetime of the marker
            marker.lifetime = Duration(sec=1, nanosec=0)  # Marker lasts for 1 second
            marker_array.markers.append(marker)
        self.human_goals.publish(marker_array)


    def publish_position_marker(self):
        marker_array = MarkerArray()  
        count = len(self.agents.x)
        x_pos = self.agents.x
        y_pos = self.agents.y

        # Predefined color palette
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
            marker.pose.position.x = x_pos[human_id]  # x position
            marker.pose.position.y = y_pos[human_id]  # y position
            marker.pose.position.z = 0.0  # z position (assumed flat plane)
            marker.scale.x = 0.2  # Sphere size in x
            marker.scale.y = 0.2  # Sphere size in y
            marker.scale.z = 0.1  # Sphere size in z
            marker.color.a = 1.0  # Transparency

            # Assign color from palette or cycle through it
            color = colors[human_id % len(colors)]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            # Set lifetime of the marker
            marker.lifetime = Duration(sec=1, nanosec=0)  # Marker lasts for 1 second
            marker_array.markers.append(marker)

        self.human_positions.publish(marker_array)
    
    
    def publish_velocity_marker(self):

        marker_array = MarkerArray()  
        count = len(self.agents.x)
        x_pos  = self.agents.x
        y_pos = self.agents.y
        x_vel = self.vel.x
        y_vel = self.vel.y

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
            color = colors[human_id % len(colors)]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            # Set lifetime of the marker
            marker.lifetime = Duration(sec=1, nanosec=0)  # Marker lasts for 1 second

            marker_array.markers.append(marker)

        self.human_velocities.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)

    goal_predictor = GoalPredictor()

    rclpy.spin(goal_predictor)
    goal_predictor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
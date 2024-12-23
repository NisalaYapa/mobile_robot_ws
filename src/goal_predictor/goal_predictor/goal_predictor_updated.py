import rclpy
import numpy                as np
import matplotlib.pyplot    as plt
from scipy.stats    import norm
from rclpy.node     import Node
from smrr_interfaces.msg import Entities


class GoalPredictor(Node):
    def __init__(self):
        super().__init__('goal_predictor')
        self.pos_subscription = self.create_subscription(
            Entities,
            'map_data',
            self.predictor_callback,
            10)
        
        self.pos_publisher  = self.create_publisher(Entities,'/pos', 10)
        self.vel_publisher  = self.create_publisher(Entities,'/vel', 10)
        self.goal_publisher = self.create_publisher(Entities,'/goals', 10)

        self.pos_subscription  

        self.pedestrian_pos = [] 
        self.pedestrian_vel = []
        self.path_buffer    = 5         # Number of past readings kept for each agent
        self.dt             = 0.2       # Position publisher rate
        self.sigma_phi      = 0.1

        self.max_agent_buffer = 12

        self.destinations = np.array([[5.0, 8.0], 
           [2.3, 1.4], 
           [3.2, 7.6], 
           [1.5, 4.8], 
           [6.1, 2.9], 
           [0.4, 9.3], 
           [8.0, 3.7], 
           [5.6, 1.1], 
           [9.5, 2.2], 
           [4.0, 0.8]])


        self.agents = Entities()
        self.vel    = Entities()
        self.goals  = Entities()

    def predictor_callback(self, msg):
        self.agents          = msg   
      
        self.vel.count       = self.agents.count
        self.vel.x           = [0.0]*self.vel.count  
        self.vel.y           = [0.0]*self.vel.count 
        
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
                
                self.vel.x[j] = ((self.pedestrian_pos[j][-2] + self.pedestrian_pos[j][-4]) - (self.pedestrian_pos[j][-6] + self.pedestrian_pos[j][-8]))/self.dt
                self.vel.y[j] = ((self.pedestrian_pos[j][-1] + self.pedestrian_pos[j][-3]) - (self.pedestrian_pos[j][-5] + self.pedestrian_pos[j][-7]))/self.dt  # Velocity calculation
                
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


def main(args=None):
    rclpy.init(args=args)

    goal_predictor = GoalPredictor()

    rclpy.spin(goal_predictor)
    goal_predictor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
import numpy as np
import matplotlib.pyplot    as plt
import matplotlib.cm        as cm # Color Map
from rclpy.node     import Node
from std_msgs.msg   import Float32MultiArray

class CrowdFlowPublisher(Node):

    def __init__(self):
        super().__init__('crowd_flow_publisher')
        self.publisher_     = self.create_publisher(Float32MultiArray, 'map_data', 10)
        timer_period        = 0.2  # seconds
        self.timer          = self.create_timer(timer_period, self.timer_callback)

        self.num_agents         = 10   
        self.goal_radius        = 0.5
        self.time_step          = 0.05   # Simulation time
        self.agent_radius       = 0.4
        self.max_speed          = 1.0
        self.num_obstacles      = 20
        self.obstacle_radius    = 0.4
        self.avoidance_strength = 0.9 

        np.random.seed(47) #20 #33
        self.positions  = np.random.rand(self.num_agents, 2) * 10
        self.goals      = np.random.rand(self.num_agents, 2) * 10
        self.obstacles  = np.random.rand(self.num_obstacles, 2) * 10

        #############################################   Encoding system for data transfer ####################################################
        #     [number of agents, number of obstacles, agent-01 X pos, agent-01 Y pos, ..., obstacle-01 X pos, obstacle-02 Y pos, ...]
        #
        ######################################################################################################################################
        #TODO: Change the input data stream to agents and goals. Remove obstacles data.
        self.mapData    = np.zeros(2 + 4*self.num_agents)  # num_goals = num_agents (Only used for validation)
        self.mapData[0] = self.num_agents
        self.mapData[1] = self.num_obstacles
        
        # Store paths for each agent
        self.paths = [self.positions[i:i+1].tolist() for i in range(self.num_agents)]

        # Visualization setup
        plt.ion()  # Activate interactive mode
        fig, ax = plt.subplots()
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)

        colors = cm.get_cmap('tab10', self.num_agents)  # Get 10 distinct colors

        self.agent_circles      = [self.create_circle(self.positions[i, 0], self.positions[i, 1], self.agent_radius, colors(i)) for i in range(self.num_agents)]
      
        goals = []
        for i in range(self.num_agents):
            goal = plt.Circle(self.goals[i], self.goal_radius, color='green')
            goals.append(goal)
            ax.add_patch(goals[i])
        
        obstacles = []
        for j in range(self.num_obstacles):
            obstacle = plt.Circle(self.obstacles[j], self.obstacle_radius, color='black')
            obstacles.append(obstacle)
            ax.add_patch(obstacles[j])
            

        # Add labels inside the circles
        self.goal_labels    = [ax.text(self.goals[i, 0], self.goals[i, 1], str(i), color='black', ha='center', va='center') for i in range(self.num_agents)]
        self.agent_labels   = [ax.text(self.positions[i, 0], self.positions[i, 1], str(i), color='black', ha='center', va='center') for i in range(self.num_agents)]

        # Initialize scatter plots 
        self.path_dots = [ax.scatter([], [], c=colors(i), s=10, alpha=0.8) for i in range(self.num_agents)]


    def timer_callback(self):
        for j in range(self.num_agents):
            self.mapData[2*j+2:2*j+4] = self.paths[j][-1]           # Add latest positions of human agents to mapData starting from 2nd position
        self.mapData[2+2*self.num_agents:] = self.goals.flatten()   # Adding the goals to last part of mapData
        msg = Float32MultiArray()
        msg.data = tuple(self.mapData)
        self.publisher_.publish(msg)
        self.crowd_flow()    # For visualization

    def create_circle(self, x, y, radius, color):
        return plt.Circle((x, y), radius, color=color, fill=False, lw=2)
    
    def check_obstacle_collision(self, position, obstacle, obstacle_radius):
        distance = np.linalg.norm(position - obstacle)
        return distance < obstacle_radius*2   # obstacle_radius + agent_radius

    # Adjust the agent's direction to avoid obstacles
    def avoid_obstacles(self, position, direction, obstacles):
        avoidance_vector = np.zeros(2)
        for obstacle in obstacles:
            if self.check_obstacle_collision(position, obstacle, self.obstacle_radius):
                avoidance_dir = position - obstacle
                avoidance_distance = np.linalg.norm(avoidance_dir)
                if avoidance_distance > 0:
                    avoidance_dir /= avoidance_distance  # Normalize | Unit vector
                    # Add to the avoidance vector with weight proportional to proximity
                    avoidance_vector += avoidance_dir * (1 / avoidance_distance) * self.avoidance_strength

        # Modify the direction based on the avoidance vector
        adjusted_direction = direction + avoidance_vector # direction: direction to GOAL
        if np.linalg.norm(adjusted_direction) > 0:
            adjusted_direction /= np.linalg.norm(adjusted_direction)  # Normalize
        
        return adjusted_direction

    def update_positions(self, positions, goals):
        for i in range(self.num_agents):
            # Vector towards the goal
            direction = goals[i] - positions[i]
            distance_to_goal = np.linalg.norm(direction)
            
            # Normalize direction
            if distance_to_goal > 0:
                direction /= distance_to_goal
            
            # Adjust the direction to avoid obstacles
            direction = self.avoid_obstacles(positions[i], direction, self.obstacles)
            
            # Move towards the goal (or adjusted direction if avoiding obstacles)
            new_position = positions[i] + direction * self.max_speed * self.time_step
            
            # Check for collisions with other agents
            for j in range(self.num_agents):
                if i != j:
                    distance = np.linalg.norm(new_position - positions[j])
                    if distance < self.agent_radius * 2:  
                        avoidance_direction = new_position - positions[j]
                        avoidance_direction /= np.linalg.norm(avoidance_direction)
                        new_position += avoidance_direction * (self.agent_radius * 2 - distance)  # To change the strength

            self.positions[i] = new_position
            self.paths[i].append(new_position.tolist())  
    
    def crowd_flow(self):
        self.update_positions(self.positions, self.goals)
        
        for i in range(self.num_agents):
            self.agent_circles[i].center = self.positions[i]
            self.agent_labels[i].set_position(self.positions[i])

            path_positions = np.array(self.paths[i])
            num_path_dots = len(path_positions)
            fade = np.linspace(0, 1, num_path_dots)  # Gradual fade effect
            self.path_dots[i].set_offsets(path_positions)
            self.path_dots[i].set_alpha(fade)

        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    crowd_flow_publisher = CrowdFlowPublisher()
    rclpy.spin(crowd_flow_publisher)
    crowd_flow_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

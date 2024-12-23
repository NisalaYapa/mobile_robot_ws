import rclpy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm # Color Map
from rclpy.node import Node
#from std_msgs.msg import Float32MultiArray
from smrr_interfaces.msg import Entities

class CrowdFlowPublisher(Node):

    def __init__(self):
        super().__init__('crowd_flow_publisher')
        self.publisher_ = self.create_publisher(Entities, 'map_data', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.num_agents = 10   
        self.goal_radius = 0.5
        self.time_step = 0.05   # Simulation time
        self.agent_radius = 0.4
        self.max_speed = 1.0
        self.num_obstacles = 5
        self.obstacle_radius = 0.4
        self.avoidance_strength = 0.9 

        np.random.seed(58)
        self.positions  = np.random.rand(self.num_agents, 2) * 10
        #self.goals      = np.random.rand(self.num_agents, 2) * 10
        self.obstacles  = np.random.rand(self.num_obstacles, 2) * 10
        self.goals = np.array([[5.0, 8.0], 
           [2.3, 1.4], 
           [3.2, 7.6], 
           [1.5, 4.8], 
           [6.1, 2.9], 
           [0.4, 9.3], 
           [8.0, 3.7], 
           [5.6, 1.1], 
           [9.5, 2.2], 
           [4.0, 0.8]])


        # self.mapData = np.zeros(2 + 4 * self.num_agents)  
        # self.mapData[0] = self.num_agents
        # self.mapData[1] = self.num_obstacles
        
        self.paths = [self.positions[i:i+1].tolist() for i in range(self.num_agents)]

        # Visualization setup
        plt.ion()  # Activate interactive mode
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Crowd flow simulator")
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(0, 10)

        colors = cm.get_cmap('tab10', self.num_agents)  # Get 10 distinct colors

        self.agent_circles = [self.create_circle(self.positions[i, 0], self.positions[i, 1], self.agent_radius, colors(i)) for i in range(self.num_agents)]
      
        self.goals_patches = []
        for i in range(self.num_agents):
            goal = plt.Circle(self.goals[i], self.goal_radius, color='green', alpha=0.5)
            self.goals_patches.append(goal)
            self.ax.add_patch(goal)

        self.obstacles_patches = []
        for j in range(self.num_obstacles):
            obstacle = plt.Circle(self.obstacles[j], self.obstacle_radius, color='black', alpha=0.3)
            self.obstacles_patches.append(obstacle)
            self.ax.add_patch(obstacle)  # Adding the obstacles

        self.goal_labels = [self.ax.text(self.goals[i, 0], self.goals[i, 1], str(i), color='black', ha='center', va='center') for i in range(self.num_agents)]
        self.agent_labels = [self.ax.text(self.positions[i, 0], self.positions[i, 1], str(i), color='black', ha='center', va='center') for i in range(self.num_agents)]

        self.path_dots = [self.ax.scatter([], [], c=colors(i), s=30, alpha=0.8) for i in range(self.num_agents)]

        # For arrows showing moving direction
        self.arrows = [self.ax.arrow(self.positions[i, 0], self.positions[i, 1], 0.1, 0.1, head_width=0.2, color=colors(i)) for i in range(self.num_agents)]

        # Footprints for agents
        self.footprints = [self.ax.scatter([], [], c=colors(i), s=50, marker=(3, 0, 30), alpha=0.5) for i in range(self.num_agents)]


    def timer_callback(self): 
        msg = Entities()
        msg.count = self.num_agents
        msg.x = []
        msg.y = []
        for j in range(self.num_agents):
            msg.x.append(self.paths[j][-1][0])
            msg.y.append(self.paths[j][-1][1])

        self.publisher_.publish(msg)
        self.crowd_flow()

    def create_circle(self, x, y, radius, color):
        return plt.Circle((x, y), radius, color=color, fill=False, lw=2)

    def check_obstacle_collision(self, position, obstacle, obstacle_radius):
        distance = np.linalg.norm(position - obstacle)
        return distance < obstacle_radius * 2

    def avoid_obstacles(self, position, direction, obstacles):
        avoidance_vector = np.zeros(2)
        for obstacle in obstacles:
            if self.check_obstacle_collision(position, obstacle, self.obstacle_radius):
                avoidance_dir = position - obstacle
                avoidance_distance = np.linalg.norm(avoidance_dir)
                if avoidance_distance > 0:
                    avoidance_dir /= avoidance_distance  
                    avoidance_vector += avoidance_dir * (1 / avoidance_distance) * self.avoidance_strength

        adjusted_direction = direction + avoidance_vector  
        if np.linalg.norm(adjusted_direction) > 0:
            adjusted_direction /= np.linalg.norm(adjusted_direction)  
        return adjusted_direction

    def update_positions(self, positions, goals):
        for i in range(self.num_agents):
            direction = goals[i] - positions[i]
            distance_to_goal = np.linalg.norm(direction)

            if distance_to_goal > 0:
                direction /= distance_to_goal

            direction = self.avoid_obstacles(positions[i], direction, self.obstacles)
            new_position = positions[i] + direction * self.max_speed * self.time_step

            for j in range(self.num_agents):
                if i != j:
                    distance = np.linalg.norm(new_position - positions[j])
                    if distance < self.agent_radius * 2:
                        avoidance_direction = new_position - positions[j]
                        avoidance_direction /= np.linalg.norm(avoidance_direction)
                        new_position += avoidance_direction * (self.agent_radius * 2 - distance)

            self.positions[i] = new_position
            self.paths[i].append(new_position.tolist())

    def crowd_flow(self):
        self.update_positions(self.positions, self.goals)

        for i in range(self.num_agents):
            self.agent_circles[i].center = self.positions[i]
            self.agent_labels[i].set_position(self.positions[i])

            path_positions = np.array(self.paths[i])
            reduced_path_positions = path_positions[::3]  # Reduce the number of dots for the path
            self.path_dots[i].set_offsets(reduced_path_positions)

            # Update the direction arrows to point to the current movement direction
            if len(self.paths[i]) > 1:
                current_direction = self.positions[i] - self.paths[i][-2]
                self.arrows[i].remove()  # Remove the old arrow
                self.arrows[i] = self.ax.arrow(self.positions[i, 0], self.positions[i, 1], current_direction[0], current_direction[1], head_width=0.2, color=self.agent_circles[i].get_edgecolor())

            # Update the footprints around agents
            self.footprints[i].set_offsets(np.array(self.paths[i])[-4:])  # Show last few positions for footprint effect

        # Redraw the obstacles
        for j in range(self.num_obstacles):
            self.obstacles_patches[j].center = self.obstacles[j]  # Ensure the obstacle position is updated
            
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

import logging
import numpy as np
from .orca import ORCA
import rvo2
from .state_plus import *
import yaml
import os


class ORCAPlusAll(ORCA):
    """
    Template to implement a teleoperation.
    This placeholder is a copy of crowd_sim_plus.envs.policy.linear
    """

    def __init__(self, time_step, time_horizon):
        super().__init__()        
        # Environment-related variables
        self.time_step = time_step
        print(self.time_step)
        self.time_horizon = time_horizon


        
    def predictAll(self, state):
        
        """
        Function to get action array for robot and Humans using ORCA
        """
        
        self_state = state.self_state
        params = self.neighbor_dist, self.max_neighbors, self.time_horizon, self.time_horizon_obst
        #logging.info('[NisalaOrca] Config {:} = {:}'.format('Running ORCA', self_state.radius))
        
        #logging.info("time_step: %s, params: %s, radius: %s, max_speed: %s", self.time_step, params, self.radius, self.max_speed)


        if self.sim is not None and self.sim.getNumAgents() != len(state.human_states) + 1:
            del self.sim
            self.sim = None
        if self.sim is None:
            self.sim = rvo2.PyRVOSimulator(self.time_step, *params, self.radius, self.max_speed)
            for i, line in enumerate(state.static_obs):
                self.sim.addObstacle(line)
            if state.static_obs:
                self.sim.processObstacles()

            self.sim.addAgent(self_state.position, *params, self_state.radius + 0.01 + self.safety_space,
                              self_state.v_pref, self_state.velocity)
            for human_state in state.human_states:
                self.sim.addAgent(human_state.position, *params, human_state.radius + 0.01 + self.safety_space,
                                  self.max_speed, human_state.velocity)
        else:
            #logging.info("Have a simulation")
            self.sim.setAgentPosition(0, self_state.position)
            self.sim.setAgentVelocity(0, self_state.velocity)
            for i, human_state in enumerate(state.human_states):
                self.sim.setAgentPosition(i + 1, human_state.position)
                self.sim.setAgentVelocity(i + 1, human_state.velocity)

        # Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal.
        velocity = np.array((self_state.gx - self_state.px, self_state.gy - self_state.py))
        speed = np.linalg.norm(velocity)
        epsilon = 1e-3
        pref_vel = velocity / speed * (self_state.v_pref - epsilon) if speed > (self_state.v_pref - epsilon) else velocity

        # Perturb a little to avoid deadlocks due to perfect symmetry.
        # perturb_angle = np.random.random() * 2 * np.pi
        # perturb_dist = np.random.random() * 0.01
        # perturb_vel = np.array((np.cos(perturb_angle), np.sin(perturb_angle))) * perturb_dist
        # pref_vel += perturb_vel

        self.sim.setAgentPrefVelocity(0, tuple(pref_vel))
        for i, human_state in enumerate(state.human_states):
            # unknown goal position of other humans
            self.sim.setAgentPrefVelocity(i + 1, (0, 0))

        self.sim.doStep()
        
        
    
        
        action_array = []
        
        #logging.info('[NisalaOrca] Config {:} = {:}'.format('RobotPositionX', self_state.px))
        #logging.info('[NisalaOrca] Config {:} = {:}'.format('RobotPositionY', self_state.py))
        #logging.info('[NisalaOrca] Config {:} = {:}'.format('RobotVelX', self_state.vx))
        #logging.info('[NisalaOrca] Config {:} = {:}'.format('RobotVelY', self_state.vy))
        
        for i  in range(len(state.human_states)+1):
            action1 = (self.sim.getAgentVelocity(i))
            action_array.append(action1)

        self.last_state = state
        self.sim = None # need this for SB3 saving

        return action_array
        
    def predictAllForTimeHorizon(self, state):
        """
        Function to get action array for robot and Humans using ORCA for all the time horizon.
        Outputs: Array(size(time_horizon, num_agents, state)) with format for the state ((self.px, self.py, self.vx, self.vy, self.radius))
        """
        predicted_states = []
        predicted_actions = []

        current_state = state
        
        
        
        
        """
        #For plotting the initial state of all agents. If you use this code segment change the time horizon of Plot.py to time_horizon + 1
        
        cur_state = []
        cur_action = []

        # Store the updated robot state (without goal info, since it may be ObservableState)
        robot_current_state = (current_state.self_state.px, current_state.self_state.py, current_state.self_state.vx, current_state.self_state.vy, current_state.self_state.radius)
        cur_state.append(robot_current_state)
        rob_action = (current_state.self_state.vx, current_state.self_state.vy)
        cur_action.append(rob_action)

        # Update human states based on the predicted actions
        for i, hum in enumerate(current_state.human_states):
            # Update velocities dynamically at each time step
            human_current_state = (hum.px, hum.py, hum.vx, hum.vy, hum.radius)
            cur_state.append(human_current_state)
            hum_action = (hum.vx, hum.vy)
            cur_action.append(hum_action)
            
        predicted_states.append(cur_state)
        predicted_actions.append(cur_action)
        """
        
        #logging.info(f"human state_current {current_state.human_states[0]}")
        


        for t in range(self.time_horizon):
            # Get the actions for all agents at the current timestep using ORCA
            actions = self.predictAll(current_state)
            #logging.info(f"currentState {current_state.self_state.vx, current_state.self_state.vy}")
            #logging.info(f"actions {actions}")

            next_state = []

            # Update the robot state based on the predicted action
            robot_action = actions[0]
            robot_next_px = current_state.self_state.px + robot_action[0] * self.time_step
            robot_next_py = current_state.self_state.py + robot_action[1] * self.time_step
            robot_next_vx = robot_action[0]
            robot_next_vy = robot_action[1]

            # Store the updated robot state (without goal info, since it may be ObservableState)
            robot_next_state = (robot_next_px, robot_next_py, robot_next_vx, robot_next_vy, current_state.self_state.radius)
            next_state.append(robot_next_state)

            # Update human states based on the predicted actions
            for i, human_action in enumerate(actions[1:]):
                human_state = current_state.human_states[i]
                human_next_px = human_state.px + human_action[0] * self.time_step
                human_next_py = human_state.py + human_action[1] * self.time_step
                human_next_vx = human_action[0]
                human_next_vy = human_action[1]

                # Update velocities dynamically at each time step
                human_next_state = (human_next_px, human_next_py, human_next_vx, human_next_vy, human_state.radius)
                next_state.append(human_next_state)

            # Store the predicted state and actions for this timestep
            predicted_states.append(next_state)
            predicted_actions.append(actions)

            # Update the current state to be the next state for the next iteration
            current_state = self.update_state_with_predicted(current_state, next_state)
            
        #logging.info(f"agents {predicted_states}")
        
        #logging.info(f"{predicted_states}")
            
        return predicted_states, predicted_actions


    
    def update_state_with_predicted(self, current_state, predicted_state):
        """
        Update the current state with the predicted state. This function is needed to keep track of the new state
        after applying the ORCA predictions, so the subsequent predictions use the updated state.
        
        """
        #logging.info(f"Predicted State {predicted_state}")
        # Update the robot state
        current_state.self_state.px = predicted_state[0][0]
        current_state.self_state.py = predicted_state[0][1]
        current_state.self_state.vx = predicted_state[0][2]
        current_state.self_state.vy = predicted_state[0][3]
        current_state.self_state.theta = np.arctan2(predicted_state[0][3], predicted_state[0][2])
        #current_state.self_state.radius = current_state.self_state.radius
        #current_state.self_state.gx = current_state.self_state.gx
        #current_state.self_state.gy = current_state.self_state.gy

        # Update human states
        for i in range(len(current_state.human_states)):
            current_state.human_states[i].px = predicted_state[i+1][0]
            current_state.human_states[i].py = predicted_state[i+1][1]
            current_state.human_states[i].vx = predicted_state[i+1][2]
            current_state.human_states[i].vy = predicted_state[i+1][3]
            current_state.human_states[i].theta = np.arctan2(predicted_state[i+1][3], predicted_state[i+1][2])
            #current_state.human_states[i].radius = current_state.self_state.radius

            

        return current_state

# Crowd Navigation using Model Predictive Control (MPC)

This repository contains the implementation of a crowd navigation system leveraging Model Predictive Control (MPC). The system enables a mobile robot to navigate through dynamic environments with dense human crowds while avoiding collisions and efficiently reaching its goal.

---

## Features

- **Crowd-Aware Navigation**: Incorporates crowd dynamics using ORCA for human trajectory prediction.
- **Optimization-Based Control**: Utilizes MPC for real-time decision-making based on predicted human positions and robot states.
- **Dynamic Environment Adaptability**: Reacts to real-time changes in the environment.
- **ROS 2 Integration**: Fully integrated with ROS 2 for simulation and deployment on robotic platforms.
- **Visualization Tools**: Includes tools for visualizing the robot’s trajectory and human interactions in CrowdSim and RViz2.

---

## Project Components

1. **MPC Algorithm**:
   - Formulates an optimization problem to minimize control effort, collision risk, and goal distance.
   - Implements cost functions:
     - Control effort cost
     - Human collision cost
     - Distance to goal
     - Terminal cost
   - Built using Python and CasADi.

2. **Human Prediction**:
   - Leverages the ORCA (Optimal Reciprocal Collision Avoidance) algorithm for predicting human trajectories.

3. **Simulation**:
   - Tested in the CrowdSim environment with dense crowd scenarios.
   - Integrated with Gazebo for realistic robot navigation testing.

4. **ROS 2 Package**:
   - Includes ROS 2 nodes for reading environment states, predicting actions, and publishing control commands at 10 Hz.
   - Designed for differential drive robots.
   - Global planner using A* integrated with the local planner for smooth navigation.

---

## Requirements

- **Python**: 3.8 or later
- **ROS 2**: Humble Hawksbill
- **Gazebo**: Simulation environment
- **Dependencies**:
  - CasADi
  - pyrvo2
  - NumPy
  - Matplotlib

---

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/your-username/crowd-navigation.git
   cd crowd-navigation

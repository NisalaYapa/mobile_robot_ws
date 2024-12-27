# Crowd Navigation using Model Predictive Control (MPC)

This is the ROS2 workspace of the SANSAR the mobile robot receptionist 2.0 with fully merged crowd navigation nodes.

---

## Steps

### launching the robot into Gazebo
```bash
ros2 launch smrr_description gazebo_gpu.launch.py
```


### vizualization in RViz2
```bash
rviz2 
```


### Openning SLAM toolbox
```bash
ros2 launch smrr_localization local_localization.launch.py 
```

### Running the Bag File
For the convinience, here is using a bag file as a human data publisher (This is recorded by running several nodes such as YOLO and human position estimators.
```bash
cd /home/nisala/mobile_robot_ws/src/human_data_buffer/human_data_buffer
ros2 bag play rosbag2_2024_12_27-00_09_10 
```

### Human Buffer Package

#### Nodes:

1. **`human_data_pub`**  
   A test human data publisher. This node is not required when the bag file is running.  

   **Topics**:  
   - **`/object_tracker/laser_data_array`**: Publishes test human data.  

2. **`human_data_extracter`**  
   This node calculates the velocities of each human agent when raw human position data is received.  

   **Topics**:  
   - **`/object_tracker/laser_data_array`**: Subscribes to human data.  
   - **`velocity_class_data`**: Publishes human positions, velocities, and classes.  

   **Markers**:  
   - **`raw_positions`**: Plots `/object_tracker/laser_data_array` (input positions).  
   - **`positions_latest`**: Publishes human positions.  
   - **`velocities_latest`**: Publishes human velocities.  

   
5. **`human_data_buffer`**
   Stores 10 values for eac human agent and calculates human motion statistics

```bash
ros2 run human_data_buffer human_data_pub 
ros2 run human_data_buffer human_data_extracter 
ros2 run human_data_buffer human_data_buffer  
```

### Prefferd Velocity Package
This is for generating a value for the preffered human velocity of each human agent using actual motion and motion details according to the human class (kid, adult, old, disabled) with integrating a Kalman Filter
```bash
ros2 run preffered_velocity_prediction preffered_velocity
```


### Goal Predictor Package
This is a node to predict the posible human destination according to their motion.
```bash
ros2 run goal_predictor goal_predictor
```

### Human Data Smoothing (Kalman Filter)

The Kalman Filter is used to smooth human positions and velocities, providing accurate data for the crowd navigation MPC.

#### Nodes:
1. **`human_kf`**  
   The primary Kalman Filter node that processes and smoothens human motion data.

2. **`kf_no_kf`**  
   A testing node that visualizes the performance of the Kalman Filter. It compares human motion predictions with and without the Kalman Filter using ORCA.


```bash
ros2 run smrr_crowdnav human_kf
ros2 run smrr_crowdnav kf_no_kf 
```

### Crowd Naavigatation Action Server
There are several versions of Crowd Mavigtion MPC

1. **`control_node_pubsub`**
  This the publisher subscriber archtecture of the MPC (No need to run the goal client)
3. **`control_node`**
   This is the latest version of MPC action server architecture
5. **`control_node_waypoint`**
   This is a testing node integrating waypoint following. (Merged into the control node)
7. **`control_node_basic_action_server`**
   This is a testing node to thest the action server architecture. (Merged into the control node)
9. **`control_node_laser`**
    This is a testing node to check the computational complexity when the raw lidar data is used as static obstacles (Need to be merged into the control node)

```bash
ros2 run smrr_crowdnav control_node
ros2 run smrr_crowdnav control_node_pubsub
ros2 run smrr_crowdnav control_node_waypoint
ros2 run smrr_crowdnav control_node_basic_action_server
ros2 run smrr_crowdnav control_node_laser
```

### Crowd Navigation Goal Client
```bash
ros2 run smrr_crowdnav goal_client
```


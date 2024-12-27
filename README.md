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

### Runnig the Bag File
For the convinience, here is using a bag file as a human data publisher (This is recorded by running several nodes such as YOLO and human position estimators.
```bash
cd /home/nisala/mobile_robot_ws/src/human_data_buffer/human_data_buffer
ros2 bag play rosbag2_2024_12_27-00_09_10 
```

### Human Buffer Package
human_data_pub : Test human data publisher (No need to run, when the bag file is running)
human_data_extracter : Calculating velocities of each hman agent when the raw human positiin data is received
human_data_buffer : Stores 10 values for eac human agent and calculates human motion statistics

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
```bash
ros2 run goal_predictor goal_predictor
```

### Kalman Filter for Smoothen the human data
```bash
ros2 run smrr_crowdnav human_kf
ros2 run smrr_crowdnav kf_no_kf 
```

### Crowd Naavigatation Action Server
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


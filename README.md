# Crowd Navigation using Model Predictive Control (MPC)

This is the ROS2 workspace of the SANSAR the mobile robot receptionist 2.0 with fully merged crowd navigation nodes.

---

## Steps

### launching the robot into Gazebo
```bash
ros2 launch smrr_description gazebo_gpu.launch.py
```


### vizualization in RViz2
rviz2 


### Openning SLAM toolbox
ros2 launch smrr_localization local_localization.launch.py 

### Runnig the Bag File
cd /home/nisala/mobile_robot_ws/src/human_data_buffer/human_data_buffer
ros2 bag play rosbag2_2024_12_27-00_09_10 

### Human Buffer Package
ros2 run human_data_buffer human_data_pub 
ros2 run human_data_buffer human_data_extracter 
ros2 run human_data_buffer human_data_buffer  

### Prefferd Velocity Package
ros2 run preffered_velocity_prediction preffered_velocity


### Goal Predictor Package
ros2 run goal_predictor goal_predictor

### Kalman Filter for Smoothen the human data
ros2 run smrr_crowdnav human_kf
ros2 run smrr_crowdnav kf_no_kf 

### Crowd Naavigatation Action Server
ros2 run smrr_crowdnav control_node
ros2 run smrr_crowdnav control_node_pubsub
ros2 run smrr_crowdnav control_node_waypoint
ros2 run smrr_crowdnav control_node_basic_action_server
ros2 run smrr_crowdnav control_node_laser


### Crowd Navigation Goal Client
ros2 run smrr_crowdnav goal_client


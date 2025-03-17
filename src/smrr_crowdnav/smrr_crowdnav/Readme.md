ros2 topic pub /goal std_msgs/msg/Float32MultiArray "{data: [2.0, 3.5]}" --once



ros2 topic pub /cancel_goal std_msgs/msg/Bool "{data: true}" --once

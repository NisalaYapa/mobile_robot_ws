from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'model', '--spawn-file=/home/sadeep/robot_recep_ws/src/gazebo_sfm_plugin-galactic/media/models/collision_box.sdf', '--model-name=collision_box'],
            output='screen'),
    ])

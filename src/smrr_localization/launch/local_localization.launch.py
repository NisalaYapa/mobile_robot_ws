from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("smrr_localization"), "config", "modified_ekf.yaml")],
    )

    start_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py" )
        ),
        launch_arguments={
            'slam_params_file':os.path.join(get_package_share_directory("smrr_multinav"), "config", "mapper_params_online_async.yaml"),
            'use_sim_time':'true'
        }.items()
    )


    return LaunchDescription([
        #robot_localization,
        start_slam_toolbox
 
    ])
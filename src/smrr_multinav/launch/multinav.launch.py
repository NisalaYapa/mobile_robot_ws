import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    
    multinav_node = Node(
        package="smrr_multinav",
        executable="multinav",
        name="multinav_node",
        parameters=[{
            "location_file": os.path.join(get_package_share_directory("smrr_multinav"), "config", "locations.yaml")
        }]
    )

    return LaunchDescription([
        multinav_node
    ])
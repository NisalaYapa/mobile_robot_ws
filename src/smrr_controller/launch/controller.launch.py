from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():

    robot_description = ParameterValue(
        Command(["xacro ",
                 os.path.join(get_package_share_directory("smrr_description"), "urdf", "smrr_description.urdf")
                 ]
                 ), 
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace='driver',
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager"
        ],
    )

    twist_mux = Node(
       package="twist_mux",
       executable="twist_mux",
       parameters=[os.path.join(get_package_share_directory("smrr_controller"),"config","twist_mux.yaml")],
       remappings=[('/cmd_vel_out', '/cmd_vel_out_unstamped')]
    )

    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        remappings=[('/cmd_vel_in', '/cmd_vel_out_unstamped'),
                    ('/cmd_vel_out', '/diff_drive_controller/cmd_vel')]
    )

    # Node fails in multi-floor navigation
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("smrr_localization"), "config", "modified_ekf.yaml")],
    )


    return LaunchDescription([
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        diff_drive_controller,
        twist_mux,
        twist_stamper

        #robot_localization
    ])
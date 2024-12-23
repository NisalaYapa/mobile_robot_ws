import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Environment variables for GPU usage
    env_use_nvidia_gpu = [
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')
    ]

    smrr_description = get_package_share_directory("smrr_description")
    smrr_description_prefix = get_package_prefix("smrr_description")

    model_path = os.path.join(smrr_description_prefix, "share")
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        smrr_description, "urdf", "smrr_description.urdf"),
                                      description="Absolute path to robot urdf file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time":True}
                    ],
    )

    # gazebo_world = os.path.join(get_package_share_directory("gazebo_sfm_plugin"), "worlds", "empty_file.world")

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")),
        # launch_arguments={
        #     "world": gazebo_world,
        #     "server_required": "True",
        #     "verbose": "True"
        # }.items()
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        )
    )

    spawn_robot = Node(package="gazebo_ros", executable="spawn_entity.py",
                        arguments=["-entity", "smrr",
                                   "-topic", "robot_description",
                                   "-x", "0", "-y", "0", "-z", "0.024",
                                  ],
                        output="screen"
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("smrr_controller"), "launch", "controller.launch.py")
        )
    )

    return LaunchDescription([
        *env_use_nvidia_gpu,
        env_var,
        model_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        controllers
    ])
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
    # env_use_nvidia_gpu = [
    #     SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
    #     SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')
    # ]

    env_use_intel_gpu = [
        # Ensures hardware acceleration (if needed, remove if it works without)
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '0'),
        # Forces Mesa loader to use the "iris" driver, which is optimized for Intel Iris GPUs
        SetEnvironmentVariable('MESA_LOADER_DRIVER_OVERRIDE', 'iris'),
        # Set the path to the GPU driver libraries
        SetEnvironmentVariable('LIBGL_DRIVERS_PATH', '/usr/lib/x86_64-linux-gnu/dri')
    ]


    smrr_description = get_package_share_directory("smrr_description")
    smrr_description_prefix = get_package_prefix("smrr_description")
    gazebo_sfm_plugin = get_package_share_directory("gazebo_sfm_plugin")

    model_path = os.path.join(smrr_description_prefix, "share")
    model_path_ = os.path.join(gazebo_sfm_plugin, "media", "models")          # not finding the models ??

    model_path += pathsep + "/home/nisala/mobile_robot_ws/src/gazebo_sfm_plugin-galactic/media/models"    # change the path accordingly
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

    gazebo_world_arg = DeclareLaunchArgument("world", default_value=os.path.join(get_package_share_directory("gazebo_sfm_plugin"), "worlds", "crowdnav.world"),
                                                description="Gazebo world file name")
    # gazebo_world = os.path.join(get_package_share_directory("gazebo_sfm_plugin"), "worlds", "small_house.world")
    gazebo_world = LaunchConfiguration("world")

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")),
        launch_arguments={
            "world": gazebo_world,
            "server_required": "True",
            "verbose": "True"
        }.items()
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        )
    )

    spawn_robot = Node(package="gazebo_ros", executable="spawn_entity.py",
                        arguments=["-entity", "smrr",
                                   "-topic", "robot_description",
                                   "-x", "-3", "-y", "0", "-z", "0.02"
                                  ],
                        output="screen"
    )

    joystick_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("smrr_controller"), "launch", "joystick_teleop.launch.py")
        )
    )


    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("smrr_controller"), "launch", "controller.launch.py")
        )
    )

    return LaunchDescription([
        # *env_use_nvidia_gpu,
        *env_use_intel_gpu,
        env_var,
        gazebo_world_arg,
        model_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        controllers,
        joystick_control
    ])

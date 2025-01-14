import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from os import pathsep
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    smrr_description = get_package_share_directory("smrr_description")
    smrr_description_prefix = get_package_prefix("smrr_description")
    gazebo_model_path_prefix = get_package_share_directory("gazebo_sfm_plugin")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    # Environment variables for GPU usage
    env_use_nvidia_gpu = [
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')
    ]

    # setting gazebo model path for simulator environment models and description models
    # model_path = os.path.join(gazebo_model_path_prefix, "media", "models")
    # model_path += pathsep + os.path.join(smrr_description_prefix, "share")
    model_path = "/home/nisala/mobile_robot_ws/src/gazebo_sfm_plugin-galactic/media/models"
    # model_path += pathsep + "/home/nisala/fyp_ws/install/smrr_description"
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # gazzebo world argument
    gazebo_world_arg = DeclareLaunchArgument(name="gazebo_world",
                                             default_value=os.path.join(gazebo_model_path_prefix, "worlds", "crowdnav.world"),
                                             description="path to gazebo world")

    # gazebo world file path
    world_path = os.path.join(get_package_share_directory('gazebo_sfm_plugin'), 'worlds', 'empty_file.world')
    # world_path = LaunchConfiguration("gazebo_world")

    # setting description path and argument
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        smrr_description, "urdf", "smrr_description.urdf"),
                                      description="Absolute path to robot urdf file")
    
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    # robot state publihser node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time":True}
                    ],
    )

    # Start Gazebo server with GPU settings
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")),
        launch_arguments={
            "world": world_path,
            "server_required": "True",
            "verbose": "True"
        }.items()
    )

    # Start Gazebo client with GPU settings
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py"))
    )

     # Spawn the robot entity
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-entity", "smrr", "-topic", "robot_description",
                   "-x", "-3", "-y", "0", "-z", "0"]
    )
    
    return LaunchDescription([
        *env_use_nvidia_gpu,
        env_var,
        gazebo_world_arg,
        model_arg,
        robot_state_publisher_node,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot,
        
    ]
    )


def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd


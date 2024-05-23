import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    zavaquin_description_dir = get_package_share_directory("zavaquin_description")
    zavaquin_description_share = os.path.join(get_package_prefix("zavaquin_description"), "share")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    model_path = os.path.join(zavaquin_description_dir, "urdf", "zavaquin.urdf.xacro")
    model_arg = DeclareLaunchArgument(
        name="ZavaQuin",
        default_value=model_path,
        description="Absolute path to robot urdf file"
    )

    world_file_name = 'ramel.world'
    world_path = os.path.join(zavaquin_description_dir, 'worlds', world_file_name)
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load'
    )

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", zavaquin_description_share)

    zavaquin_description = ParameterValue(Command(["xacro ", LaunchConfiguration("ZavaQuin")]), value_type=str)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": zavaquin_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")),
        launch_arguments={'world': world_path}.items()
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "ZavaQuin", "-topic", "robot_description"],
        output="screen"
    )

    return LaunchDescription([
        env_var,
        model_arg,
        declare_world_cmd,
        LogInfo(msg=["Using robot model path: ", model_path]),
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot
    ])

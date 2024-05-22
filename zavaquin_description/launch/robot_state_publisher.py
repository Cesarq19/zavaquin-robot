import os
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Launch config variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    
    zavaquin_description_dir = get_package_share_directory('zavaquin_description')

    # Argumento de lanzamiento para especificar la ubicación del archivo URDF del robot
    model_arg = DeclareLaunchArgument(
        name="model_zavaquin",
        default_value=os.path.join(zavaquin_description_dir, "urdf", "zavaquin.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    robot_description_config = ParameterValue(Command(['xacro ', LaunchConfiguration("model_zavaquin"), ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time]), value_type=str)

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) time if true')

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        name='use_ros2_control',
        default_value='False',
        description='Use ros2_control if true')
    
    # Start robot state publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(model_arg)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)

    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)

    return ld

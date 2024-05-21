import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch import LaunchDescription


def generate_launch_description():
    zavaquin_bringup_dir = get_package_share_directory("zavaquin_bringup")
    zavaquin_description_dir = get_package_share_directory("zavaquin_description")
    zavaquin_teleop_dir = get_package_share_directory("zavaquin_teleop")
    zavaquin_navigation_dir = get_package_share_directory("zavaquin_navigation")

    controller_params_file = os.path.join(zavaquin_bringup_dir, "config", "controllers.yaml")
    ekf_params_file = os.path.join(zavaquin_teleop_dir, "config", "ekf.yaml")

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_robot_localization = LaunchConfiguration("use_robot_localization")

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        name="use_ros2_control",
        default_value="True",
        description="Use ros2_control if true",
    )

    declare_use_robot_localization_cmd = DeclareLaunchArgument(
        name="use_robot_localization",
        default_value="True",
        description="Use robot_localization package if true",
    )

    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(zavaquin_description_dir, "launch", "robot_state_publisher_launch.py")]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_ros2_control": use_ros2_control,
        }.items(),
    )

    # Start robot state publisher
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(zavaquin_description_dir, "launch", "robot_state_publisher_launch.py")]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_ros2_control": use_ros2_control,
        }.items(),
    )

    robot_description = Command(
        ["ros2 param get --hide-type /robot_state_publisher robot_description"]
    )

    # Launch controller manager
    start_controller_manager_cmd = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_params_file],
    )

    # Delayed controller manager action
    start_delayed_controller_manager = TimerAction(
        period=2.0, actions=[start_controller_manager_cmd]
    )

    # Spawn diff_controller
    start_diff_controller_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
    )

    # Delayed diff_drive_spawner action
    start_delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager_cmd,
            on_start=[start_diff_controller_cmd],
        )
    )

    # Spawn joint_state_broadcaser
    start_joint_broadcaster_cmd = Node(
        # condition=IfCondition(use_ros2_control),
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delayed joint_broadcaster_spawner action
    start_delayed_joint_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager_cmd,
            on_start=[start_joint_broadcaster_cmd],
        )
    )

    # Spawn imu_sensor_broadcaser
    start_imu_broadcaster_cmd = Node(
        # condition=IfCondition(use_ros2_control),
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster"],
    )

    # Delayed imu_broadcaster_spawner action
    start_delayed_imu_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager_cmd,
            on_start=[start_imu_broadcaster_cmd],
        )
    )

    # Start robot localization using an Extended Kalman Filter
    start_robot_localization_cmd = Node(
        condition=IfCondition(use_robot_localization),
        package="robot_localization",
        executable="ekf_node",
        parameters=[ekf_params_file],
    )

    # Start joystick node
    start_joystick_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(zavaquin_teleop_dir, "launch", "joystick_launch.py")]
        )
    )

    # Start rplidar node
    start_rplidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(zavaquin_bringup_dir, "launch", "rplidar_launch.py")]
        )
    )

    # Start camera node
    start_camera_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(zavaquin_bringup_dir, "launch", "camera_launch.py")]
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(declare_use_robot_localization_cmd)

    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_delayed_controller_manager)
    ld.add_action(start_delayed_diff_drive_spawner)
    ld.add_action(start_delayed_joint_broadcaster_spawner)
    ld.add_action(start_delayed_imu_broadcaster_spawner)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_joystick_cmd)

    return ld
    pass
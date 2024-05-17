# Librerias necesarias
import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Función principal para generar la descripción del lanzamiento
def generate_launch_description():
    # Directorio de descripción de Roomba
    roomba_description_dir = get_package_share_directory("roomba_description")
    
    # Argumento de lanzamiento para especificar la ubicación del archivo URDF del robot
    model_arg = DeclareLaunchArgument(
        name="roomba",
        default_value=os.path.join(roomba_description_dir, "urdf", "roomba_description.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )
    
    # Configuración del parámetro del nodo del robot_state_publisher
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("roomba")]), value_type=str)
    
    # Nodo para publicar el estado del robot
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Nodo para la interfaz gráfica del publicador de estados de las articulaciones
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # Nodo para RViz con la configuración del archivo .rviz proporcionado
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(roomba_description_dir, "rviz", "description.rviz")]
    )

    # Devuelve la descripción del lanzamiento con los elementos especificados
    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
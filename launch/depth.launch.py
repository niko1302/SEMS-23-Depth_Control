from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction)
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    
    arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(arg)
    
    # --------------------------------
    # -- Get Launch Parameter Files --
    # --------------------------------
    package_path = get_package_share_path('depth_control')

    calculator_file_path = str(package_path / 'config/calculator_params.yaml')
    calculator_file_args = DeclareLaunchArgument(
        'calculator_config_file',
        default_value=calculator_file_path)
    launch_description.add_action(calculator_file_args)

    controller_file_path = str(package_path  / 'config/controller_params.yaml')
    controller_file_args = DeclareLaunchArgument(
        'controller_config_file',
        default_value=controller_file_path)
    launch_description.add_action(controller_file_args)

    setpoint_publisher_file_path = str(package_path / 'config/setpoint_publisher_params.yaml')
    setpoint_publisher_file_args = DeclareLaunchArgument(
        'setpoint_publisher_config_file',
        default_value=setpoint_publisher_file_path)
    launch_description.add_action(setpoint_publisher_file_args)

    # -----------------------
    # -- Set Node settings --
    # -----------------------
    calculator = Node(
        executable='depth_calculator.py',
        package='depth_control',
        parameters=[LaunchConfiguration('calculator_config_file')])
    
    controller = Node(
        executable='depth_controller.py',
        package='depth_control',
        parameters=[LaunchConfiguration('controller_config_file')])
    
    setpoint_publisher = Node(
        executable='depth_setpoint.py',
        package='depth_control',
        parameters=[LaunchConfiguration('setpoint_publisher_config_file')])

    # -----------------------------------
    # -- Group Nodes and set Namespace --
    # -----------------------------------
    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        calculator,
        controller,
        setpoint_publisher,
    ])

    launch_description.add_action(group)
    return launch_description

#!/usr/bin/env python3
"""
Launch RViz visualization for School Project robotic cart.

This launch file sets up the complete visualization environment for the robot,
including robot state publisher, joint state publisher, and RViz2. It handles loading
and processing of URDF/XACRO files and controller configurations.

:author: vamichrom
:date: February 15, 2025
"""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def process_ros2_controllers_config(context):
    """Process the ROS 2 controller configuration yaml file before loading the URDF.

    This function reads a template configuration file, replaces placeholder values
    with actual configuration, and writes the processed file to both source and
    install directories.

    Args:
        context: Launch context containing configuration values

    Returns:
        list: Empty list as required by OpaqueFunction
    """

    # Get the configuration values
    prefix = LaunchConfiguration('prefix').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    enable_odom_tf = LaunchConfiguration('enable_odom_tf').perform(context)

    home = str(Path.home())

    # Define both source and install paths
    src_config_path = os.path.join(
        home,
        '/home/ladmin/Documents/projects/ROS2/school_project_ws/src/robo_cart/robo_cart_description/config',
        robot_name
    )
    install_config_path = os.path.join(
        home,
        '/home/ladmin/Documents/projects/ROS2/school_project_ws/install/robo_cart_description/share/robo_cart_description/config',
        robot_name
    )

    # Read from source template
    template_path = os.path.join(src_config_path, 'ros2_controllers_template.yaml')
    with open(template_path, 'r', encoding='utf-8') as file:
        template_content = file.read()

    # Create processed content (leaving template untouched)
    processed_content = template_content.replace('${prefix}', prefix)
    processed_content = processed_content.replace(
        'enable_odom_tf: true', f'enable_odom_tf: {enable_odom_tf}')

    # Write processed content to both source and install directories
    for config_path in [src_config_path, install_config_path]:
        os.makedirs(config_path, exist_ok=True)
        output_path = os.path.join(config_path, 'ros2_controllers.yaml')
        with open(output_path, 'w', encoding='utf-8') as file:
            file.write(processed_content)

    return []


# Define the arguments for the XACRO file
ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='robo_cart',
                          description='Name of the robot'),
    DeclareLaunchArgument('prefix', default_value='',
                          description='Prefix for robot joints and links'),
    DeclareLaunchArgument('use_gazebo', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to use Gazebo simulation'),
    DeclareLaunchArgument('enable_odom_tf', default_value='true',
                          choices=['true', 'false'],
                          description='Enable odometry transform broadcasting via ROS 2 Control')
]


def generate_launch_description():
    """Generate the launch description for the robot visualization.

    This function sets up all necessary nodes and parameters for visualizing
    the robot in RViz, including:
    - Robot state publisher for broadcasting transforms
    - Joint state publisher for simulating joint movements
    - RViz for visualization

    Returns:
        LaunchDescription: Complete launch description for the visualization setup
    """
    # Define filenames
    urdf_package = 'robo_cart_description'
    urdf_filename = 'robot.urdf.xacro'
    rviz_config_filename = 'robo_cart_description.rviz'

    # Set paths to important files
    pkg_share_description = FindPackageShare(urdf_package)
    default_urdf_model_path = PathJoinSubstitution(
        [pkg_share_description, 'urdf', urdf_filename])
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share_description, 'rviz', rviz_config_filename])

    # Launch configuration variables
    jsp_gui = LaunchConfiguration('jsp_gui')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    robot_description_content = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'robot_name:=', LaunchConfiguration('robot_name'), ' ',
        'prefix:=', LaunchConfiguration('prefix'), ' ',
        'use_gazebo:=', LaunchConfiguration('use_gazebo')
    ]), value_type=str)

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content}])

    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(jsp_gui))

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(jsp_gui))

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])

    # Create the launch description and populate
    ld = LaunchDescription(ARGUMENTS)

    # Process the controller configuration before starting nodes
    ld.add_action(OpaqueFunction(function=process_ros2_controllers_config))

    # Declare the launch options
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add any actions
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld

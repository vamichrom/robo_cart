import os

from ament_index_python import get_package_share_path

from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, ThisLaunchFileDir
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    package_name = 'robo_cart_description'
    
    urdf_path = os.path.join(
        get_package_share_path(package_name),
        'urdf',
        'robot.urdf.xacro'
    )
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]))

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )
    
    controller_params_file = os.path.join(
        get_package_share_path(package_name),
        'config',
        'ros2_controllers.yaml'
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )
    
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller"]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    # When controller_manager starts launch mecanum_drive_spawner
    delayed_mecanum_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[mecanum_drive_controller_spawner],
        )
    )
    
    # When controller_manager starts launch joint_state_broadcaster
    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        delayed_controller_manager,
        delayed_mecanum_drive_controller_spawner,
        delayed_joint_state_broadcaster_spawner,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/lidar.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/camera.launch.py'])
        ),
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/joystick.launch.py'])
        #), # I don't need joystick on robot machine, it will be run on dev machine (on ThinkPad)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/obj_tracker.launch.py'])
        ),
    ])

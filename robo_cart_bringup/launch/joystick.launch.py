from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    start_joy_node_cmd = Node(
            package='joy',
            executable='joy_node',
            parameters=[{'use_sim_time': use_sim_time}],
         )

    start_teleop_node_cmd = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            parameters=[{'use_sim_time': use_sim_time}],
         )

    start_usb_gamepad_node_cmd = Node(
            package='usb_gamepad',
            executable='usb_gamepad',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/mecanum_drive_controller/cmd_vel')]
         )

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'))
    ld.add_action(start_joy_node_cmd)
    ld.add_action(start_teleop_node_cmd)
    ld.add_action(start_usb_gamepad_node_cmd)

    return ld

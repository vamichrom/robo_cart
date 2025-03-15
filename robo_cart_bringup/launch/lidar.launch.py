import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    ld = LaunchDescription()

    start_x3_ydlidar_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'ydlidar_ros2_driver', 'ydlidar_launch.py'],
        output='screen'
    )

    ld.add_action(start_x3_ydlidar_cmd)
    
    return ld

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    start_v4l2_hd_webcam_c525_cmd = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen',
        parameters=[{'video_device': '/dev/webcam',
                     'camera_frame_id': 'camera_link_optical',
                     'output_encoding': 'yuv422_yuy2',
                     'pixel_format': 'YUYV',
                     'image_size': [640,480]}]
    )

    ld.add_action(start_v4l2_hd_webcam_c525_cmd)
    
    return ld

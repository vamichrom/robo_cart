# Robotic cart school project

A prototype of robotic cart for school project running ROS 2 Jazzy on a Raspberry Pi 5

![Robotic cart](https://github.com/vamichrom/robo_cart/blob/33f686ad1c9d103fd74e0b665e3640d5055037f3/docs/jpeg/robo_cart.jpg)

## Usage

1. [Install Ubuntu Noble 24.04](https://www.google.com/search?q=Install+Ubuntu+Noble)
2. [Install ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
3. [Create a workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
4. Clone repository into workspace

        cd <workspace-directory>/src
        git clone https://github.com/vamichrom/robo_cart.git

5. [Configure dependencies](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html)

        cd <workspace-directory>
        rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO

6. Build the workspace

        cd <workspace-directory>
        colcon build

    Result:

        Starting >>> robo_cart_description
        Starting >>> obj_tracker
        Starting >>> mecanum_drive_controller
        Starting >>> robo_cart_gazebo
        Starting >>> ros2_control_arduino_hw
        Starting >>> usb_gamepad
        Starting >>> ydlidar_ros2_driver
        Finished <<< robo_cart_description [1.19s]                                                        
        Starting >>> robo_cart_bringup
        Finished <<< ros2_control_arduino_hw [1.19s]                                                         
        Finished <<< robo_cart_gazebo [1.24s]
        Finished <<< mecanum_drive_controller [1.27s]
        Finished <<< ydlidar_ros2_driver [1.30s]                                        
        Finished <<< robo_cart_bringup [0.24s]                                                                         
        Finished <<< usb_gamepad [3.05s]                                            
        Finished <<< obj_tracker [3.11s]          

        Summary: 8 packages finished [3.69s]

7. [Source the workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay) into every new terminal session or configure ~/.bashrc

        cd <workspace-directory>
        . install/setup.bash

8. Use the application

    Rviz:

        ros2 launch robo_cart_description display.launch.xml
        
    ![Rviz Gif](https://github.com/vamichrom/robo_cart/blob/33f686ad1c9d103fd74e0b665e3640d5055037f3/docs/gif/rviz.gif)

    Gazebo:

        ros2 launch robo_cart_bringup robo_cart.launch.xml

    ![Gazebo Gif](https://github.com/vamichrom/robo_cart/blob/33f686ad1c9d103fd74e0b665e3640d5055037f3/docs/gif/gazebo.gif)

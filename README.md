# Robotic cart school project

A prototype of robotic cart for school project running ROS 2 Jazzy on a Raspberry Pi 5

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
        Starting >>> mecanum_drive_controller
        Starting >>> robo_cart_gazebo
        Starting >>> ros2_control_arduino_hw
        Starting >>> usb_gamepad
        Finished <<< ros2_control_arduino_hw [0.66s]
        Finished <<< robo_cart_description [0.70s]
        Starting >>> robo_cart_bringup
        Finished <<< mecanum_drive_controller [0.72s]
        Finished <<< robo_cart_gazebo [0.76s]
        Finished <<< robo_cart_bringup [0.18s]
        Finished <<< usb_gamepad [2.15s]

        Summary: 6 packages finished [2.40s]

7. [Source the workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay) into every new terminal session or configure ~/.bashrc

        cd <workspace-directory>
        . install/setup.bash

8. Use the application

    Rviz:

        ros2 launch robo_cart_description display.launch.xml

        ![Rviz Gif](https://github.com/vamichrom/robo_cart/blob/8aca91e14bb38d08dbbbebfd5badc79b85019049/docs/gifs/rviz.gif)

    Gazebo:

        ros2 launch robo_cart_bringup robo_cart.launch.xml

        ![Gazebo Gif](https://github.com/vamichrom/robo_cart/blob/8aca91e14bb38d08dbbbebfd5badc79b85019049/docs/gifs/gazebo.gif)

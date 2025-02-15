# Robotic Cart School Project

A prototype of robotic cart for school project running ROS 2 Jazzy on a Raspberry Pi 5

## Usage

1. Install Ubuntu Noble 24.04
2. Install ROS 2 Jazzy
3. Create workspace
4. Clone repository into workspace
5. Build the workspace

Rviz:
```bash
ros2 launch robo_cart_description display.launch.xml
```

Gazebo:
```bash
ros2 launch robo_cart_bringup robo_cart.launch.xml
```

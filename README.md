# robotics_project
LapinAMK Robotics and AI project work spring 2024

Important! If there are any iron or rolling nodes in the same network, VSLAM won't work. Use seperate WiFi

1. Edit bashrc file to automatically source ros2 distro

gedit ~/.bashrc
Add to the end of file :
source /opt/ros/humble/setup.bash
source /microros_ws/install/setup.bash

2. Creating ros2 packages PYTHON
ros2 pkg create name --build-type ament_python --dependencies rclpy

Creating ros2 packages C++
ros2 pkg create name --build-type ament_cmake --dependencies rclpy

3. Run micro ros agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

Run RQT GUI
rqt


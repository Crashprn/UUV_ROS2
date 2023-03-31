#!/bin/bash

uuvIP=$(hostname -I)

echo "UUV IP address: $uuvIP"

echo -e "\nSetting up UUV ROS workspace and starting arduino interface..."
cd UUV_ROS2/ros_ws
# colcon build
. install/setup.bash
ros2 launch nodes_py arduino_joy.launch.py

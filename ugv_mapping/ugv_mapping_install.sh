#!/bin/bash

ls /dev/tty* | grep ttyUSB1 #check /dev/ttyUSB1
cd sllidar_ros2/scripts
source create_udev_rules.sh
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py

cd ~/ugv_ws
colcon build --packages-select ugv_mapping
source install/setup.bash
ros2 launch ugv_mapping mapping.launch.py

cd ~/ugv_ws
colcon build --packages-select ugv_mapping
source install/setup.bash
ros2 launch ugv_mapping gazebo.launch.py

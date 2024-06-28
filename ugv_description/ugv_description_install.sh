#!/bin/bash

cd ~/ugv_ws && colcon build --packages-select ugv_description && source install/setup.bash && ros2 launch ugv_description display.launch.py

cd ~/ugv_ws
colcon build --packages-select ugv_description
source install/setup.bash
ros2 launch ugv_description gazebo.launch.py
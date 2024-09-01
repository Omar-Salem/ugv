#!/bin/bash

cd ~/ugv_ws
colcon build --packages-select ugv_nav
source install/setup.bash
ros2 launch ugv_nav nav.launch.py

cd ~/ugv_ws && colcon build --packages-select ugv_nav && source install/setup.bash && ros2 launch ugv_nav gazebo.launch.py

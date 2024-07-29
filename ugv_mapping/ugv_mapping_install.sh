#!/bin/bash

cd ~/ugv_ws && colcon build --packages-select ugv_mapping && source install/setup.bash && ros2 launch ugv_mapping mapping.launch.py

cd ~/ugv_ws
colcon build --packages-select ugv_mapping
source install/setup.bash
ros2 launch ugv_mapping gazebo.launch.py

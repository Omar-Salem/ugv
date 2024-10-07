#!/bin/bash

# Real
cd ~/ugv_ws && colcon build --packages-select ugv_mapping && source install/setup.bash && ros2 launch ugv_mapping mapping.launch.py


# Sim
cd ~/ugv_ws && colcon build --packages-select ugv_mapping && source install/setup.bash && ros2 launch ugv_mapping gazebo.launch.py

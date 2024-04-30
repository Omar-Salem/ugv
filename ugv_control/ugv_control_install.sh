#!/bin/bash
colcon build --packages-select ugv_control
source install/setup.bash
ros2 launch ugv_control gazebo_control.launch.py
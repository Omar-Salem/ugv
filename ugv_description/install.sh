#!/bin/bash
colcon build --packages-select ugv_description
source install/setup.bash
ros2 launch ugv_description display.launch.py
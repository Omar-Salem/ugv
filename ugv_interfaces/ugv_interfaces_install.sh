#!/bin/bash
cd ~/ugv_ws
colcon build --packages-select ugv_interfaces
source install/setup.bash
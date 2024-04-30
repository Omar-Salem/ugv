#! /bin/bash

mkdir -p ugv_ws/src
cd ugv_ws || exit
(
cd src || exit
git clone https://github.com/Omar-Salem/ugv.git
)
rm -rf build/ install/ log/
colcon build
source install/setup.bash
ros2 launch ugv_control control.launch.py
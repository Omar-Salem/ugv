#! /bin/bash

mkdir -p ugv_ws/src
cd $_
git clone https://github.com/Omar-Salem/ugv.git

mv two_wheels/* .
rm -rf two_wheels
cd ..
rm -rf build/ install/ log/
colcon build
source install/setup.bash && ros2 launch two_wheels_core two_wheels.launch.py
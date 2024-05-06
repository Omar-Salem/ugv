#!/bin/bash

cd ~/ugv_ws
colcon build --packages-select ugv_control
source install/setup.bash
ros2 launch ugv_control control.launch.py

cd ~/ugv_ws
colcon build --packages-select ugv_control
source install/setup.bash
ros2 launch ugv_control gazebo.launch.py


ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "
  linear:
    x: 20"

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
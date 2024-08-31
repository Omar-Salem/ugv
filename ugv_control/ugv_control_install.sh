#!/bin/bash

# terminal 1
cd ~/ugv_ws && colcon build --packages-select ugv_control && source install/setup.bash && ros2 launch ugv_control control.launch.py
####################### O R ##############################
cd ~/ugv_ws && colcon build --packages-select ugv_control && source install/setup.bash && ros2 launch ugv_control gazebo.launch.py

# terminal 2
ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "
twist:
  linear:
    x: 0.1"

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=True
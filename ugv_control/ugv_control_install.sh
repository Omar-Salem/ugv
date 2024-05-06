#!/bin/bash

colcon build --packages-select ugv_control
source install/setup.bash
ros2 launch ugv_control control.launch.py

colcon build --packages-select ugv_control
source install/setup.bash
ros2 launch ugv_control gazebo.launch.py


ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "
  linear:
    x: 0.5"
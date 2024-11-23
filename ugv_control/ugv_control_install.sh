#!/bin/bash

# UGV
cd ~/ugv_ws && colcon build --packages-select ugv_control && source install/setup.bash && ros2 launch ugv_control control.launch.py

# Laptop
cd ~/ugv_ws && colcon build --packages-select ugv_control && source install/setup.bash && ros2 launch ugv_control control.launch.py micro_ros_port:=/dev/ttyUSB0 use_gui:=True

# Sim
cd ~/ugv_ws && colcon build --packages-select ugv_control && source install/setup.bash && ros2 launch ugv_control gazebo.launch.py

# Sanity check
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: auto, twist: {angular: {z: 0.1}}}"


ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=True
#!/bin/bash

# flash basic first
# check connections
# check battery is on

# flash esp32

# terminal 1
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0


# terminal 2
cd ~/ugv_ws
colcon build --packages-select ugv_interfaces
source ~/microros_ws/install/local_setup.bash
source install/setup.bash 

ros2 topic list -t | grep ugv/motors #check for /ugv/motors_cmd and /ugv/motors_state

# base, both same direction
ros2 topic pub -r 10 /ugv/motors_cmd ugv_interfaces/msg/MotorsOdom "left: 200.0
right: 200.0" 

ros2 topic pub --once /ugv/motors_cmd ugv_interfaces/msg/MotorsOdom "left: 0.0
right: 0.0" 

# reverse direction
ros2 topic pub -r 10 /ugv/motors_cmd ugv_interfaces/msg/MotorsOdom "left: -200.0
right: 200.0" 

# reverse right direction
ros2 topic pub -r 10 /ugv/motors_cmd ugv_interfaces/msg/MotorsOdom "left: 200.0
right: -200.0" 

# reverse both
ros2 topic pub -r 10 /ugv/motors_cmd ugv_interfaces/msg/MotorsOdom "left: -200.0
right: -200.0" 
#!/bin/bash

# flash basic first
# check connections
# check battery is on

# clean platformio
# flash esp32
pio pkg install # Install dependencies
pio run --target upload # Flash the firmware


ls /dev/ttyUSB* | grep ttyUSB0 #check /dev/ttyUSB0
lsusb | grep CP210x

# terminal 1
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# terminal 2
cd ~/ugv_ws
colcon build --packages-select ugv_interfaces
source install/setup.bash 
ros2 topic list -t | grep ugv/motors #check for /ugv/motors_cmd and /ugv/motors_state

# forward
ros2 topic pub --once /ugv/motors_cmd ugv_interfaces/msg/MotorsOdom "rear_left: 6.28
rear_right: 6.28
front_left: 6.28
front_right: 6.28" 

# stop
ros2 topic pub --once /ugv/motors_cmd ugv_interfaces/msg/MotorsOdom "rear_left: 0.0
rear_right: 0.0
front_left: 0.0
front_right: 0.0" 

# backward
ros2 topic pub -r 10 /ugv/motors_cmd ugv_interfaces/msg/MotorsOdom "rear_left: -6.28
rear_right: -6.28
front_left: -6.28
front_right: -6.28" 

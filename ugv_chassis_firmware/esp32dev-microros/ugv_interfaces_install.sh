#!/bin/bash

# flash basic first
# check connections
# check battery is on

# flash esp32

# terminal 1
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0


# terminal 2
source ~/microros_ws/install/local_setup.bash
source install/setup.bash 

ros2 topic list -t | grep ugv/motors #check for cmd and state topics

ros2 topic pub -r 10 /ugv_interfaces/motors_cmd ugv_interfaces/msg/MotorsOdom "left:
  velocity: 10
right:
  velocity: 10"
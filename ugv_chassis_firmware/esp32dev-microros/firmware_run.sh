#!/bin/bash

# flash basic first
# check connections
# check battery is on

# clean platformio
# flash esp32
cd ~/ugv_ws
colcon build --packages-select ugv_interfaces
source ~/microros_ws/install/local_setup.bash
source install/setup.bash 
pio lib install # Install dependencies
pio run # Build the firmware
pio run --target upload # Flash the firmware

# terminal 1
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO serial --dev /dev/ttyUSB0 -v6


# terminal 2
cd ~/ugv_ws
colcon build --packages-select ugv_interfaces
source ~/microros_ws/install/local_setup.bash
source install/setup.bash 

ros2 topic list -t | grep ugv/motors #check for /ugv/motors_cmd and /ugv/motors_state

# both same direction
ros2 topic pub -r 10 /ugv/motors_cmd ugv_interfaces/msg/MotorsOdom "rear_left: 6.28
rear_right: 6.28" 

# stop
ros2 topic pub --once /ugv/motors_cmd ugv_interfaces/msg/MotorsOdom "rear_left: 0.0
rear_right: 0.0" 

# reverse rear_left direction
ros2 topic pub -r 10 /ugv/motors_cmd ugv_interfaces/msg/MotorsOdom "rear_left: -6.28
rear_right: 6.28" 

# reverse rear_right direction
ros2 topic pub -r 10 /ugv/motors_cmd ugv_interfaces/msg/MotorsOdom "rear_left: 6.28
rear_right: -6.28" 

# reverse both
ros2 topic pub -r 10 /ugv/motors_cmd ugv_interfaces/msg/MotorsOdom "rear_left: -6.28
rear_right: -6.28" 

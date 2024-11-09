#!/bin/bash


# UGV
cd ~/ugv_ws && colcon build --packages-select ugv_nav && source install/setup.bash && ros2 launch ugv_nav nav.launch.py


# Sim
cd ~/ugv_ws && colcon build --packages-select ugv_nav && source install/setup.bash && ros2 launch ugv_nav gazebo.launch.py


ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/omar.salem/ugv_ws/src/ugv_mapping/maps/apartment/map.yaml}"

ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=/home/omar.salem/ugv_ws/src/ugv_mapping/maps/apartment/map.yaml
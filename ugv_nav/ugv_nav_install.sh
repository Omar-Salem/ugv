#!/bin/bash


# UGV
cd ~/ugv_ws && colcon build --packages-select ugv_nav && source install/setup.bash && ros2 launch ugv_nav nav.launch.py


# Sim
cd ~/ugv_ws && colcon build --packages-select ugv_nav && source install/setup.bash && ros2 launch ugv_nav gazebo.launch.py



ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{filename : '/home/omar-salem/ugv_ws/src/ugv_mapping/maps/gazebo/walls/map',match_type : 1}"


ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename : '/home/omar-salem/ugv_ws/src/ugv_mapping/maps/gazebo/walls/map'}"
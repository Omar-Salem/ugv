#!/bin/bash

# UGV
cd ~/ugv_ws && colcon build --packages-select ugv_mapping && source install/setup.bash && ros2 launch ugv_mapping mapping.launch.py


# Sim
cd ~/ugv_ws && colcon build --packages-select ugv_mapping && source install/setup.bash && ros2 launch ugv_mapping gazebo.launch.py


ros2 topic pub --once /diff_drive_controller/odom nav_msgs/msg/Odometry "{header: {frame_id: 'odom'},child_frame_id: 'base_link', pose: {pose: {orientation: {x: 0.0,y: 0.0,z: 1.0,w: 0.0}}}}"


ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{filename : '/home/omar.salem/ugv_ws/src/ugv_mapping/maps/apartment/map.yaml',match_type : 1}"


ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename : '/home/omar-salem/ugv_ws/src/ugv_mapping/maps/gazebo/walls/map'}"
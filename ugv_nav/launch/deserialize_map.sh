#!/bin/bash
MAP_PATH=$1
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{filename : '$MAP_PATH',match_type : 1}"
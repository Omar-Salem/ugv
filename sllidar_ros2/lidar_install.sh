ls /dev/ttyUSB* | grep ttyUSB1 #check /dev/ttyUSB1
sudo chmod 777 /dev/ttyUSB1
cd ~/ugv_ws/src/sllidar_ros2/scripts
chmod +x create_udev_rules.sh
./create_udev_rules.sh

#lidar
cd ~/ugv_ws && colcon build --packages-select sllidar_ros2 && source install/setup.bash && ros2 launch sllidar_ros2 sllidar_c1.launch.py

#view
cd ~/ugv_ws && colcon build --packages-select sllidar_ros2 && source install/setup.bash && ros2 launch sllidar_ros2 view_sllidar_c1.launch.py
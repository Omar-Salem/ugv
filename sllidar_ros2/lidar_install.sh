ls /dev/tty* | grep ttyUSB1 #check /dev/ttyUSB1
sudo chmod 777 /dev/ttyUSB1
cd ~/ugv_ws && colcon build --packages-select sllidar_ros2 && source install/setup.bash && ros2 launch sllidar_ros2 view_sllidar_c1.launch.py
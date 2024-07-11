## Requirements
### Jazzy
```bash
sudo apt -y update && sudo apt -y upgrade

locale  # check for UTF-8

sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings


sudo apt install software-properties-common -y
sudo add-apt-repository universe

sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt install ros-dev-tools -y
sudo apt install ros-jazzy-desktop -y

echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc 
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc 
source ~/.bashrc

sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y
````
### Packages
````
sudo apt-get install -y ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-joint-state-publisher ros-${ROS_DISTRO}-robot-state-publisher ros-${ROS_DISTRO}-xacro
sudo apt install -y ros-${ROS_DISTRO}-ros2-control
sudo apt install -y ros-${ROS_DISTRO}-ros2-controllers
sudo apt install -y ros-${ROS_DISTRO}-robot-localization
sudo apt install -y ros-${ROS_DISTRO}-navigation2
sudo apt install -y ros-${ROS_DISTRO}-nav2-bringup
sudo apt install -y ros-${ROS_DISTRO}-controller-manager
sudo apt install -y ros-${ROS_DISTRO}-rosbag2
sudo apt install -y ros-${ROS_DISTRO}-behaviortree-cpp-v3
sudo apt install -y ros-${ROS_DISTRO}-tf-transformations
sudo apt-get install -y ros-${ROS_DISTRO}-image-tools
sudo apt-get install -y ros-${ROS_DISTRO}-cartographer
sudo apt install -y ros-${ROS_DISTRO}-twist-mux
sudo apt install -y ros-${ROS_DISTRO}-angles
sudo apt install -y ros-${ROS_DISTRO}-mqtt-client
sudo apt install -y ros-${ROS_DISTRO}-launch-xml
sudo apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf2-tools
sudo apt install python3-colcon-common-extensions

sudo apt-get install -y ros-${ROS_DISTRO}-rviz2 
sudo apt install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-gazebo-ros2-control ros-${ROS_DISTRO}-joint-state-publisher-gui
export QT_QPA_PLATFORM=xcb rviz2
sudo usermod -aG dialout ${USER}
````

### Docker
````
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker ${USER}
````

### microros
````
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo apt install python3-rosdep2
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
sudo apt-get install python3-pip
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
echo 'source ~/microros_ws/install/local_setup.bash' >> ~/.bashrc 
````
## Build and run simulation

```bash
cd ugv_ws/
rm -rf build/ install/ log/
colcon build && source install/setup.bash && ros2 launch ugv_control gazebo_control.launch.py
````

## Build and run real

[//]: # (### Log into on Pi)

[//]: # (```bash)

[//]: # (ssh omar.salem@192.168.1.35)

[//]: # (```)

### Prepare docker on Pi (_first time/on code change_)
```bash
HOST=omar.salem@192.168.1.35
scp prepare_docker.sh $HOST:/tmp/ && ssh -t $HOST "sudo -s bash /tmp/prepare_docker.sh"
```

### Run microros

```bash
ssh omar.salem@192.168.1.35
docker run --rm --privileged -it -v ~/Volumes:/home/usr/ humble
#contents of run_microros.sh
```

### Run robot

```bash
ssh omar.salem@192.168.1.35
docker exec -it $(docker container ls  | grep 'humble' | awk '{print $1}') bash
#contents of run_robot.sh
```

### Test
```bash
ssh omar.salem@192.168.1.35
docker exec -it $(docker container ls  | grep 'humble' | awk '{print $1}') bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "
  linear:
    x: 0.5"

````

### Debug

- Is Esp32 LED on?
- Do Motors Have power ?
```bash

ls /dev/tty* | grep USB #sanity check to see if usb devices are reachable
ros2 topic list | grep motors #sanity check to see if /ugv/motors_* topics exist

source ~/microros_ws/install/local_setup.bash
ros2 topic echo /ugv/motors_state #sanity check to see if topics emit

ros2 topic pub -r 10 /ugv/motors_cmd ugv_interfaces/msg/MotorsOdom "front_left:
  velocity: 1.5
front_right:
  velocity: 1.5
rear_left:
  velocity: 1.5
rear_right:
  velocity: 1.5"
````

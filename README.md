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
sudo add-apt-repository universe -y

sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt install ros-dev-tools -y
sudo apt install ros-jazzy-desktop -y

echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc 
echo 'export QT_QPA_PLATFORM=xcb rviz2' >> ~/.bashrc 
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc 
source ~/.bashrc
sudo apt-get install python3-rosdep -y
echo ${${ROS_DISTRO}}
````
### Packages
````bash
sudo apt-get install -y ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-joint-state-publisher ros-${ROS_DISTRO}-robot-state-publisher ros-${ROS_DISTRO}-xacro
sudo apt install -y ros-${ROS_DISTRO}-ros2-control
sudo apt install -y ros-${ROS_DISTRO}-joint-state-publisher-gui
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
sudo apt install -y python3-colcon-common-extensions
sudo apt-get install -y ros-${ROS_DISTRO}-rviz2 
sudo apt install -y ros-${ROS_DISTRO}-ros-gz
sudo apt install -y ros-${ROS_DISTRO}-gz-ros2-control
sudo apt install vim -y
sudo usermod -aG dialout ${USER}
````

### microros
````bash
cd ~
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo apt update 
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
sudo apt-get install -y python3-pip
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
echo 'source ~/microros_ws/install/local_setup.bash' >> ~/.bashrc 
source ~/.bashrc
````

### Create workspace and clone repo
````bash
mkdir -p ~/ugv_ws/src
cd ~/ugv_ws/src 
git clone https://github.com/Omar-Salem/ugv.git .
cd ~/ugv_ws
colcon build 
````

### Run

```bash
# basic
cd ~/ugv_ws && colcon build --packages-select ugv_description && source install/setup.bash && ros2 launch ugv_description display.launch.py
```



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

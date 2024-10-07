import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "ugv_mapping"
    package_dir = FindPackageShare(package_name)
    rviz_config_file_arg = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=PathJoinSubstitution([package_dir, "config", "display.rviz"]),
    )

    rp_lidar_c1 = build_rp_lidar_c1_node()

    slam_toolbox = build_slam_toolbox_node(package_name)
    
    control = build_control_node(LaunchConfiguration("rviz_config_file"))
    
    return LaunchDescription([rviz_config_file_arg,
                              slam_toolbox, 
                              rp_lidar_c1, 
                              control
                              ])

def build_control_node(rviz_config_file):

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ugv_control"),
                    "launch",
                    "control.launch.py",
                )
            ]
        ),
        launch_arguments={
            "rviz_config_file": rviz_config_file
            }.items(),
    )
    
    return control

def build_slam_toolbox_node(package_name):
    slam_params_file = os.path.join(
                 get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml'
             )

    slam_toolbox_launch_file = [os.path.join(get_package_share_directory('slam_toolbox'), 'launch'), '/online_async_launch.py']
    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_launch_file),
        launch_arguments={ 'use_sim_time': 'False',
                          'slam_params_file':slam_params_file}.items()
             )
             
    return slam_toolbox

def build_rp_lidar_c1_node():
    rp_lidar_c1_launch_file_path = PathJoinSubstitution(
        [FindPackageShare("sllidar_ros2"), "launch", "sllidar_c1.launch.py"]
    )
    rp_lidar_c1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rp_lidar_c1_launch_file_path),
        launch_arguments={ 'serial_port': '/dev/ttyUSB0',
                          'frame_id':'lidar_link'
                          }.items()
    )
    
    return rp_lidar_c1

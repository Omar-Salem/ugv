import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def create_nav2_node(package_name):
    map_yaml_file = PathJoinSubstitution([FindPackageShare('ugv_mapping'), "maps", "apt.yaml"])
    navigation_launch_file_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'launch', 'bringup_launch.py'])
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file_path),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'True',
            'package_name': package_name
            }.items()
    )
    return nav2_bringup

def generate_launch_description():

    return LaunchDescription(
                             [
        create_nav2_node('ugv_nav'),
                              IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ugv_mapping'), 'launch', 'gazebo.launch.py'
        )])
    )
                              ]
                             )

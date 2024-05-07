import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'ugv_mapping'

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'core.launch.py'
        )]), launch_arguments={
            'package_name': package_name,
            'is_sim': 'False'
        }.items()
    )

    package_dir = FindPackageShare(package_name)

    world = PathJoinSubstitution(
        [package_dir, 'worlds', 'many_walls.world']
    )
    rviz_config_file = PathJoinSubstitution(
        [package_dir, 'config', 'display.rviz']
    )

    return LaunchDescription(
        [core,
         IncludeLaunchDescription(
             PythonLaunchDescriptionSource([os.path.join(
                 get_package_share_directory('ugv_control'), 'launch', 'gazebo.launch.py'
             )]), launch_arguments={
                 'gazebo_world': world,
                 'rviz_config_file': rviz_config_file
             }.items()
         )
         ]
    )

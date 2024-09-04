import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap

def generate_launch_description():

    nav2_params_file_path = PathJoinSubstitution(
        [FindPackageShare('ugv_nav'), 'config', 'nav2_params.yaml'])
    
    nav_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'), 'launch'), '/navigation_launch.py']),
        launch_arguments={ 'use_sim_time': 'True',
                              'params_file': nav2_params_file_path,}.items()
             )
    mapping_node = GroupAction(
        actions=[

                SetRemap(src='/cmd_vel', dst='/diff_drive_controller/cmd_vel'),

        IncludeLaunchDescription(
             PythonLaunchDescriptionSource([os.path.join(
                 get_package_share_directory('ugv_mapping'), 'launch', 'gazebo.launch.py'
             )])
         )
    ]
)
    
    return LaunchDescription(
        [mapping_node,
         nav_node
         ]
    )

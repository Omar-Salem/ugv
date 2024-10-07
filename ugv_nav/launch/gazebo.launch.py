import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node,SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    mapping = build_mapping_node()
    
    nav2_bringup = build_nav2_bringup_node()

    return LaunchDescription([mapping,
                              nav2_bringup])

def build_nav2_bringup_node():
  return  GroupAction(
    actions=[

        SetRemap(src='/cmd_vel',dst='/diff_drive_controller/cmd_vel'),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("nav2_bringup"), "launch"),
                "/bringup_launch.py",
            ]
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": os.path.join(
                get_package_share_directory("ugv_nav"),
                "config",
                "nav2_params.yaml",
            ),
            'map':os.path.join(
                    get_package_share_directory("ugv_mapping"),
                    "maps",
                    "map.yaml",
                )
        }.items(),
    )
    ]
)

def build_mapping_node():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ugv_mapping"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
        }.items(),
    )

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

    world = PathJoinSubstitution([package_dir, "worlds", "many_walls.world"])
    rviz_config_file_arg = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=PathJoinSubstitution([package_dir, "config", "display.rviz"]),
    )
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("slam_toolbox"), "launch"),
                "/online_async_launch.py",
            ]
        ),
        launch_arguments={
            "use_sim_time": "True",
            "slam_params_file": os.path.join(
                get_package_share_directory("ugv_mapping"),
                "config",
                "mapper_params_online_async.yaml",
            ),
        }.items(),
    )

    control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ugv_control"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gazebo_world": world,
            "rviz_config_file": rviz_config_file,
        }.items(),
    )

    return LaunchDescription([slam_toolbox, control_node, rviz_config_file_arg])

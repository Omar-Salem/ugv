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

    slam_toolbox = build_slam_toolbox_node(package_name)

    return LaunchDescription(
        [
            slam_toolbox,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("ugv_mapping"), "launch"
                        ),
                        "/lidar.launch.py",
                    ]
                ),
                launch_arguments={"use_sim_time": "False"}.items(),
            ),
        ]
    )


def build_slam_toolbox_node(package_name):
    slam_params_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "mapper_params_online_async.yaml",
    )

    slam_toolbox_launch_file = [
        os.path.join(get_package_share_directory("slam_toolbox"), "launch"),
        "/online_async_launch.py",
    ]
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file),
        launch_arguments={
            "use_sim_time": "False",
            "slam_params_file": slam_params_file,
        }.items(),
    )

    return slam_toolbox

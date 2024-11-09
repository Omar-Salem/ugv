import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    package_name = "ugv_mapping"
    package_dir = FindPackageShare(package_name)
    rviz_config_file_arg = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=PathJoinSubstitution([package_dir, "config", "display.rviz"]),
    )

    rp_lidar_c1 = build_rp_lidar_c1_node()

    control_nodes = build_control_node(LaunchConfiguration("rviz_config_file"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            rviz_config_file_arg,
            rp_lidar_c1,
        ]
        + control_nodes
    )


def build_control_node(rviz_config_file):

    use_sim_time = LaunchConfiguration("use_sim_time")
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
        launch_arguments={"rviz_config_file": rviz_config_file}.items(),
        condition=UnlessCondition(use_sim_time),
    )

    gazebo_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ugv_control"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={"rviz_config_file": rviz_config_file}.items(),
        condition=IfCondition(use_sim_time),
    )

    return [control, gazebo_control]


def build_rp_lidar_c1_node():
    rp_lidar_c1_launch_file_path = PathJoinSubstitution(
        [FindPackageShare("sllidar_ros2"), "launch", "sllidar_c1.launch.py"]
    )
    rp_lidar_c1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rp_lidar_c1_launch_file_path),
        launch_arguments={
            "serial_port": "/dev/ttyUSB0",
            "frame_id": "lidar_link",
        }.items(),
    )

    return rp_lidar_c1

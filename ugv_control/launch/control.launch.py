import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "ugv_control"
    share_dir = get_package_share_directory(package_name)

    controller_nodes = create_controller_nodes(package_name)

    display_node = create_display_node(share_dir)
    micro_ros = ExecuteProcess(
        cmd=[os.path.join(share_dir, "launch", "microros.sh")], output="screen"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="rviz_config_file",
                default_value=os.path.join(share_dir, "config", "display.rviz"),
            ),
            micro_ros,
            display_node,
        ]
        + controller_nodes
    )


def create_display_node(share_dir):
    xacro_file = os.path.join(share_dir, "urdf", "ugv.xacro")
    xacro_file_mapped = xacro.process_file(xacro_file, mappings={"is_sim": "false"})
    robot_urdf = xacro_file_mapped.toxml()

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ugv_description"),
                    "launch",
                    "display.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_urdf": robot_urdf,
            "use_sim_time": "False",
            "rviz_config_file": LaunchConfiguration("rviz_config_file"),
        }.items(),
    )


def create_controller_nodes(package_name):
    robot_controller_names = ["joint_state_broadcaster", "diff_drive_controller"]
    robot_controller_spawners = []
    for c in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[c],
            )
        ]
    package_dir = FindPackageShare(package_name)
    robot_controllers = PathJoinSubstitution(
        [package_dir, "config", "diff_drive_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers],
    )
    robot_controller_spawners.append(control_node)
    return robot_controller_spawners

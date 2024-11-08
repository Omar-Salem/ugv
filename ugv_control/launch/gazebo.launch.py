import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "ugv_control"
    share_dir = get_package_share_directory(package_name)
    xacro_file = os.path.join(share_dir, "urdf", "ugv.xacro")
    robot_description_config = xacro.process_file(
        xacro_file, mappings={"is_sim": "true"}
    )
    robot_urdf = robot_description_config.toxml()

    controller_nodes = create_controller_nodes()

    ugv_description_launch = [
        os.path.join(get_package_share_directory("ugv_description"), "launch"),
        "/gazebo.launch.py",
    ]

    rviz_config_file_arg = DeclareLaunchArgument(
        name="rviz_config_file",
        default_value=os.path.join(share_dir, "config", "display.rviz"),
    )
    return LaunchDescription(
        [
            rviz_config_file_arg,
            GroupAction(
                actions=[
                    SetParameter(name="use_sim_time", value=True),
                    #         Node(
                    #     package="odometry_test",
                    #     executable="square_follower",
                    # ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(ugv_description_launch),
                        launch_arguments={
                            "robot_urdf": robot_urdf,
                            "use_gui": "True",
                            "use_joint_state_publisher": "False",
                            "rviz_config_file": LaunchConfiguration("rviz_config_file"),
                        }.items(),
                    ),
                ]
                + controller_nodes
            ),
        ]
    )


def create_controller_nodes() -> list:
    """

    :rtype: list
    """
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ugv_control"),
            "config",
            "diff_drive_controllers.yaml",
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--param-file",
            robot_controllers,
        ],
    )
    return [joint_state_broadcaster_spawner, diff_drive_controller_spawner]

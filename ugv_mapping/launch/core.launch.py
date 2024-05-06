import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "is_sim",
        )]

    robot_nodes = [
        GroupAction(
            actions=[
                SetRemap(src='/cmd_vel', dst='/diff_drive_controller/cmd_vel_unstamped'),
                create_slam_toolbox_node(),
            ]
        )
    ]

    return LaunchDescription(declared_arguments +
                             robot_nodes
                             )


def create_slam_toolbox_node():

    is_sim = LaunchConfiguration('is_sim')
    package_name ='ugv_mapping'
    slam_toolbox_launch_file_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'launch', 'online_async_launch.py'])
    slam_params_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "mapper_params_online_async.yaml"])
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file_path),
        launch_arguments={'use_sim_time': is_sim, 'slam_params_file': slam_params_file}.items()
    )
    return slam_toolbox

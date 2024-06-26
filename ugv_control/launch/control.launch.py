import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'ugv_control'
    share_dir = get_package_share_directory(package_name)
    xacro_file = os.path.join(share_dir, 'urdf', 'ugv.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={'is_sim': 'false'})
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_urdf,
                'use_sim_time': False
            }
        ]
    )

    controller_nodes = create_controller_nodes(package_name, robot_urdf)

    rviz_config_file = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=os.path.join(share_dir, 'config', 'display.rviz')
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        output='screen'
    )

    return LaunchDescription(
        [
            rviz_config_file,
            robot_state_publisher_node,
            rviz_node
        ] +
        controller_nodes
        # TODO microros
        # source ~/microros_ws/install/local_setup.bash
        # ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
    )


def create_controller_nodes(package_name, robot_description_config):
    robot_controller_names = ['joint_state_broadcaster', 'diff_drive_controller']
    robot_controller_spawners = []
    for controller in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
            )
        ]
    package_dir = FindPackageShare(package_name)
    robot_controllers = PathJoinSubstitution(
        [package_dir, "config", 'diff_drive_controllers.yaml']
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[{'robot_description': robot_description_config}, robot_controllers],
    )
    robot_controller_spawners.append(control_node)
    return robot_controller_spawners

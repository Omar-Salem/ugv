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
    robot_description_config = xacro.process_file(xacro_file, mappings={'is_sim': 'true'})
    robot_urdf = robot_description_config.toxml()



    controller_nodes = create_controller_nodes()

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
            DeclareLaunchArgument(
                name='gazebo_world',
                default_value=os.path.join(share_dir, 'worlds', 'many_walls.world')
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ugv_description'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={ 'world': LaunchConfiguration('gazebo_world')}.items()
         
             ),
            rviz_config_file,
            rviz_node
        ] +
        controller_nodes
    )


def create_controller_nodes() -> list:
    """

    :rtype: list
    """
    robot_controller_names = ['joint_state_broadcaster', 'diff_drive_controller']
    robot_controller_spawners = []
    for controller in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller]
            )
        ]
    return robot_controller_spawners


    """

    :rtype: list
    """
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': LaunchConfiguration('gazebo_world')}.items()
    )
    spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ugv'],
                        output='screen')
    # launch.logging.launch_config.level = logging.WARN
    return [gazebo, spawn_entity]

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
    share_dir = get_package_share_directory('ugv_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'ugv.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={'is_sim': 'true'})
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_urdf,
                'use_sim_time': True
            }
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )


    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ugv',
            '-topic', 'robot_description'
        ],
        output='screen'
    )
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    return LaunchDescription([
            DeclareLaunchArgument(
                name='gazebo_world',
                default_value=''
            ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        urdf_spawn_node,
        rviz_node
    ]+create_gazebo_nodes())
    
def create_gazebo_nodes() -> list:
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

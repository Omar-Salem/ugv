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
    robot_urdf = robot_description_config.toprettyxml(indent='  ')

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
                name='world',
                default_value=os.path.join(share_dir, 'worlds', 'fws_robot_world.sdf')
            ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ]+create_gazebo_nodes(robot_urdf))

def create_gazebo_nodes(robot_urdf) -> list:
    """

    :rtype: list
    """    
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
         
             )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_urdf,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.07',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'fws_robot',
                   '-allow_renaming', 'false'],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    return [gazebo,gz_spawn_entity,bridge]

# def create_gazebo_nodes(robot_urdf) -> list:
    """

    :rtype: list
    """
    ros_gz_sim =get_package_share_directory('ros_gz_sim')
    gzserver_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    ros_gz_sim, 'launch'), '/gz_sim.launch.py']),
    launch_arguments={'gz_args': ['-v4 ', LaunchConfiguration('gazebo_world')], 'on_exit_shutdown': 'true'}.items()
             )
    gzclient_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': '-g -v4 '}.items()
)

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_urdf,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.07',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'ugv',
                   '-allow_renaming', 'false'],
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )
    return [gzserver_cmd, gz_spawn_entity,bridge]

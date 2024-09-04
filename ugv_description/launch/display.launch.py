from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    share_dir = get_package_share_directory('ugv_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'ugv.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={'is_sim': 'false'}).toxml()
        
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False'
    )
    rviz_config_file_arg = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=os.path.join(share_dir, 'config', 'display.rviz')
    )
    
    robot_urdf_arg = DeclareLaunchArgument(
        name='robot_urdf',
        default_value=robot_description_config
    )

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    robot_urdf = LaunchConfiguration('robot_urdf')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_urdf,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_urdf_arg,
        rviz_config_file_arg,
        rviz_node,
        robot_state_publisher_node
    ])

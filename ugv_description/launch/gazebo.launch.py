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
    share_dir = get_package_share_directory("ugv_description")
    xacro_file = os.path.join(share_dir, "urdf", "ugv.xacro")
    robot_description_config = xacro.process_file(
        xacro_file, mappings={"is_sim": "true"}
    ).toprettyxml(indent="  ")

    robot_urdf_arg = DeclareLaunchArgument(
        name="robot_urdf", default_value=robot_description_config
    )
    robot_urdf = LaunchConfiguration("robot_urdf")

    gazebo_world_arg = DeclareLaunchArgument(
                name="world",
                default_value=os.path.join(
                    get_package_share_directory("ugv_description"),
                    "worlds",
                    "walls",
                ),
            )
    
    display_launch_file = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(share_dir, "launch"), "/display.launch.py"]
                ),
                launch_arguments={
                    "use_sim_time": "True",
                    "use_gui": "True",
                    "use_joint_state_publisher": LaunchConfiguration("use_joint_state_publisher")
                    }.items(),
            )
    
    return LaunchDescription(
        [
            DeclareLaunchArgument(
        name="use_joint_state_publisher", default_value='True'
    ),
            robot_urdf_arg,
            gazebo_world_arg,
            display_launch_file,
        ]
        + create_gazebo_nodes(robot_urdf),
    )


def create_gazebo_nodes(robot_urdf) -> list:
    """

    :rtype: list
    """

    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    world = LaunchConfiguration("world")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(ros_gz_sim, "launch"), "/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", [world, ".sdf", " -v 4", " -r"])],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_urdf,
            "-name",
            "ugv",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"],
        output="screen",
    )

    return [gazebo, gz_spawn_entity, bridge]

import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch_ros.actions import SetParameter


def generate_launch_description():
    return LaunchDescription(
        [
            GroupAction(
                actions=[
                    SetParameter(name="use_sim_time", value=True),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                os.path.join(
                                    get_package_share_directory("ugv_nav"),
                                    "launch",
                                    "nav.launch.py",
                                )
                            ]
                        ),
                        launch_arguments={
                            "use_sim_time": "true",
                            "map_yaml_file": "/home/omar-salem/ugv_ws/src/ugv_mapping/maps/gazebo/walls/map.yaml",
                        }.items(),
                    ),
                ]
            ),
        ]
    )

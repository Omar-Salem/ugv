# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


from launch.actions import (
    DeclareLaunchArgument,
)


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ugv_nav"), "config", "display.rviz"]
    )

    ugv_mapping = get_package_share_directory("ugv_mapping")
    ugv_nav = get_package_share_directory("ugv_nav")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "map_yaml_file",
                default_value="/home/omar.salem/ugv_ws/src/ugv_mapping/maps/apartment/map.yaml",
                description="Automatically startup the nav2 stack",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(ugv_nav, "launch", "bringup.launch.py")]
                ),
                launch_arguments={
                    "params_file": PathJoinSubstitution(
                        [FindPackageShare("ugv_nav"), "config", "nav2_params.yaml"]
                    ),
                    "map": LaunchConfiguration("map_yaml_file"),
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(ugv_mapping, "launch", "lidar.launch.py")]
                ),
                launch_arguments={
                    "rviz_config_file": rviz_config_file,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )

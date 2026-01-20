# Copyright (c) 2026 BYU FRoSt Lab
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
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def launch_setup(context, *args, **kwargs):

    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf_file = LaunchConfiguration("urdf_file")
    auv_ns = LaunchConfiguration("auv_ns")
    play_bag_path = LaunchConfiguration("play_bag_path")
    record_bag_path = LaunchConfiguration("record_bag_path")
    compare = LaunchConfiguration("compare")

    play_bag_path_str = context.perform_substitution(play_bag_path)
    record_bag_path_str = context.perform_substitution(record_bag_path)

    coug_bringup_dir = get_package_share_directory("coug_bringup")
    coug_bringup_launch_dir = os.path.join(coug_bringup_dir, "launch")

    actions = []

    if play_bag_path_str:
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    play_bag_path_str,
                    "--clock",
                ],
            )
        )

    if record_bag_path_str:
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-a",
                    "-o",
                    record_bag_path_str,
                    "--storage",
                    "mcap",
                ],
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(coug_bringup_launch_dir, "base.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "multiagent_viz": "false",
            }.items(),
        )
    )

    auv_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_bringup_launch_dir, "auv.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "urdf_file": urdf_file,
            "auv_ns": auv_ns,
            "set_origin": "true",
            "compare": compare,
        }.items(),
    )

    agent_actions = [
        PushRosNamespace(auv_ns),
        auv_launch,
    ]

    actions.append(GroupAction(actions=agent_actions))

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (rosbag) clock if true",
            ),
            DeclareLaunchArgument(
                "urdf_file",
                default_value="urdf/bluerov2_heavy/bluerov2_heavy.urdf.xacro",
                description="URDF or Xacro file to load",
            ),
            DeclareLaunchArgument(
                "auv_ns",
                default_value="bluerov2",
                description="Namespace for the AUV (e.g. auv0)",
            ),
            DeclareLaunchArgument(
                "play_bag_path",
                default_value="",
                description="Path to play rosbag from",
            ),
            DeclareLaunchArgument(
                "record_bag_path",
                default_value="",
                description="Path to record rosbag (if empty, no recording)",
            ),
            DeclareLaunchArgument(
                "compare",
                default_value="false",
                description="Launch additional localization nodes if true",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

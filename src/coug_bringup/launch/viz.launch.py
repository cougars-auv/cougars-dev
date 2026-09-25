# Copyright 2026 BYU FROST Lab
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
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list_config = LaunchConfiguration("agent_list")
    play_bag_path = LaunchConfiguration("play_bag_path")
    start_offset = LaunchConfiguration("start_offset")
    playback_duration = LaunchConfiguration("playback_duration")
    playback_rate = LaunchConfiguration("playback_rate")
    start_paused = LaunchConfiguration("start_paused")

    play_bag_path_str = play_bag_path.perform(context)

    coug_bringup_dir = get_package_share_directory("coug_bringup")
    coug_bringup_launch_dir = os.path.join(coug_bringup_dir, "launch")

    actions: list[Action] = []

    if play_bag_path_str:
        start_paused_args = (
            ["--start-paused"] if IfCondition(start_paused).evaluate(context) else []
        )
        play_process = ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                play_bag_path_str,
                "--clock",
                "--start-offset",
                start_offset,
                "--playback-duration",
                playback_duration,
                "--rate",
                playback_rate,
                *start_paused_args,
            ],
        )
        actions.append(play_process)

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(coug_bringup_launch_dir, "base.launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "agent_list": agent_list_config,
                "record_bag_path": "",
                "enable_base_processing": "false",
                "initialize_origin": "false",
            }.items(),
        )
    )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOG_DIR", launch_config.log_dir),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "agent_list",
                default_value="[auv0]",
            ),
            DeclareLaunchArgument(
                "play_bag_path",
                default_value="",
            ),
            DeclareLaunchArgument(
                "start_offset",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "playback_duration",
                default_value="-1.0",
            ),
            DeclareLaunchArgument(
                "playback_rate",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "start_paused",
                default_value="false",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

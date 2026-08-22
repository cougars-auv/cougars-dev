# Copyright (c) 2026 BYU FROST Lab
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

import atexit
import os
import shutil
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def save_config(record_bag_path: str) -> None:
    if not record_bag_path or not os.path.isdir(record_bag_path):
        return

    config_dir = os.environ.get("CONFIG_DIR", "")
    if not config_dir or not os.path.isdir(config_dir):
        return

    dest = os.path.join(record_bag_path, "config")
    shutil.copytree(config_dir, dest, dirs_exist_ok=True)
    print(f"Config saved: {dest}")


def save_logs(record_bag_path: str) -> None:
    if not record_bag_path or not os.path.isdir(record_bag_path):
        return

    log_dir = launch_config.log_dir
    if not os.path.isdir(log_dir):
        return

    dest = os.path.join(record_bag_path, "log")
    shutil.copytree(log_dir, dest, dirs_exist_ok=True)
    print(f"Logs saved: {dest}")


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list = LaunchConfiguration("agent_list")
    lead_agent = LaunchConfiguration("lead_agent")
    enable_direct_comms = LaunchConfiguration("enable_direct_comms")
    enable_acoustic_comms = LaunchConfiguration("enable_acoustic_comms")
    record_bag_path = LaunchConfiguration("record_bag_path")

    agent_list_str = agent_list.perform(context)
    record_bag_path_str = record_bag_path.perform(context)

    agent_namespaces = yaml.safe_load(agent_list_str)
    auv_ns = agent_namespaces[0]

    coug_comms_dir = get_package_share_directory("coug_comms")
    coug_comms_launch_dir = os.path.join(coug_comms_dir, "launch")
    coug_fgo_dir = get_package_share_directory("coug_fgo")
    coug_fgo_launch_dir = os.path.join(coug_fgo_dir, "launch")
    coug_active_fgo_dir = get_package_share_directory("coug_active_fgo")
    coug_active_fgo_launch_dir = os.path.join(coug_active_fgo_dir, "launch")
    coug_visual_dvl_dir = get_package_share_directory("coug_visual_dvl")
    coug_visual_dvl_launch_dir = os.path.join(coug_visual_dvl_dir, "launch")
    coug_mapviz_dir = get_package_share_directory("coug_mapviz")
    coug_mapviz_launch_dir = os.path.join(coug_mapviz_dir, "launch")
    coug_rqt_dir = get_package_share_directory("coug_rqt")
    coug_rqt_launch_dir = os.path.join(coug_rqt_dir, "launch")

    coug_comms_base_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_comms_launch_dir, "coug_comms_base.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list,
            "lead_agent": lead_agent,
            "enable_direct_comms": enable_direct_comms,
            "enable_acoustic_comms": enable_acoustic_comms,
        }.items(),
    )

    coug_fgo_base_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_fgo_launch_dir, "coug_fgo_base.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list,
        }.items(),
    )

    coug_fgo_viz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_fgo_launch_dir, "coug_fgo_viz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list,
            "launch_rviz": "true",
            "launch_plotjuggler": "true",
        }.items(),
    )

    coug_active_fgo_viz_cmd = IncludeLaunchDescription(  # noqa: F841
        PythonLaunchDescriptionSource(
            os.path.join(coug_active_fgo_launch_dir, "coug_active_fgo_viz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "auv_ns": auv_ns,
            "launch_rviz": "true",
            "launch_plotjuggler": "true",
        }.items(),
    )

    coug_visual_dvl_viz_cmd = IncludeLaunchDescription(  # noqa: F841
        PythonLaunchDescriptionSource(
            os.path.join(coug_visual_dvl_launch_dir, "coug_visual_dvl_viz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "auv_ns": auv_ns,
        }.items(),
    )

    coug_mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_mapviz_launch_dir, "coug_mapviz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list,
        }.items(),
    )

    coug_rqt_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_rqt_launch_dir, "coug_rqt.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list,
        }.items(),
    )

    base_station_group = GroupAction(
        actions=[
            PushRosNamespace("base_station"),
            coug_comms_base_cmd,
            coug_fgo_base_cmd,
        ]
    )

    actions = [
        base_station_group,
        coug_fgo_viz_cmd,
        # coug_active_fgo_viz_cmd,
        # coug_visual_dvl_viz_cmd,
        coug_mapviz_cmd,
        coug_rqt_cmd,
    ]

    if record_bag_path_str:
        record_process = ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "-a",
                "-o",
                record_bag_path_str,
                "--storage",
                "mcap",
                "--exclude-topics",
                "/clock",
            ],
            sigterm_timeout="15",
            sigkill_timeout="15",
        )
        actions.append(record_process)

        atexit.register(save_config, record_bag_path_str)
        atexit.register(save_logs, record_bag_path_str)

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOG_DIR", launch_config.log_dir),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "agent_list",
                default_value="[auv0]",
                description=(
                    "YAML list of agent namespaces "
                    "(e.g. '[coug1sim]' or '[coug1sim, coug2sim]')"
                ),
            ),
            DeclareLaunchArgument(
                "lead_agent",
                default_value="",
                description="Namespace of the lead agent (optional)",
            ),
            DeclareLaunchArgument(
                "record_bag_path",
                default_value="",
                description="Path to record rosbag (if empty, no recording)",
            ),
            DeclareLaunchArgument(
                "enable_direct_comms",
                default_value="true",
                description="Enable direct ROS service communications",
            ),
            DeclareLaunchArgument(
                "enable_acoustic_comms",
                default_value="true",
                description="Enable acoustic communications",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

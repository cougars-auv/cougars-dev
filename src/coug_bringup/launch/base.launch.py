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

import atexit
import os
import shutil
import tempfile
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger, launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def snapshot_config() -> str:
    config_dir = os.environ.get("CONFIG_DIR", "")
    if not os.path.isdir(config_dir):
        return ""
    snapshot = tempfile.mkdtemp(prefix="config_")
    shutil.copytree(config_dir, snapshot, dirs_exist_ok=True)
    return snapshot


def save_artifacts(record_bag_path: str, config_snapshot: str) -> None:
    if not record_bag_path or not os.path.isdir(record_bag_path):
        return

    artifacts = (
        ("Config", config_snapshot, "config"),
        ("Logs", launch_config.log_dir, "log"),
    )
    for label, source, directory in artifacts:
        if os.path.isdir(source):
            destination = os.path.join(record_bag_path, directory)
            shutil.copytree(source, destination, dirs_exist_ok=True)
            get_logger("launch.user").info(f"{label} saved: {destination}")


def create_plotjuggler_config(agent_list: list[str]) -> str:
    config_dir = os.environ["CONFIG_DIR"]
    with open(os.path.join(config_dir, "gui", "plotjuggler.xml.template")) as template:
        content = template.read().replace("<agent_ns>", agent_list[0])

    with tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".xml") as rendered_config:
        rendered_config.write(content)
        return rendered_config.name


def create_rviz_config(agent_list: list[str]) -> str:
    config_dir = os.environ["CONFIG_DIR"]
    with open(os.path.join(config_dir, "gui", "rviz.rviz.template")) as template:
        config = yaml.safe_load(template)

    displays = config["Visualization Manager"]["Displays"]
    templates = [yaml.safe_dump(display, sort_keys=False) for display in displays]
    shared = [text for text in templates if "<agent_ns>" not in text]
    per_agent = [text for text in templates if "<agent_ns>" in text]

    displays[:] = [yaml.safe_load(text) for text in shared]
    for agent_ns in agent_list:
        group = [yaml.safe_load(text.replace("<agent_ns>", agent_ns)) for text in per_agent]
        if len(agent_list) > 1:
            group = [
                {
                    "Class": "rviz_common/Group",
                    "Name": agent_ns,
                    "Enabled": True,
                    "Displays": group,
                }
            ]
        displays.extend(group)

    with tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".rviz") as rendered_config:
        content = yaml.safe_dump(config, sort_keys=False)
        rendered_config.write(content.replace("<agent_ns>", agent_list[0]))
        return rendered_config.name


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list_config = LaunchConfiguration("agent_list")
    scenario_param_file = LaunchConfiguration("scenario_param_file")
    lead_agent = LaunchConfiguration("lead_agent")
    record_bag_path = LaunchConfiguration("record_bag_path")
    enable_direct_comms = LaunchConfiguration("enable_direct_comms")
    enable_acoustic_comms = LaunchConfiguration("enable_acoustic_comms")
    enable_base_processing = LaunchConfiguration("enable_base_processing")
    initialize_origin = LaunchConfiguration("initialize_origin")

    agent_list_str = agent_list_config.perform(context)
    record_bag_path_str = record_bag_path.perform(context)

    agent_list = yaml.safe_load(agent_list_str)

    coug_comms_dir = get_package_share_directory("coug_comms")
    coug_comms_launch_dir = os.path.join(coug_comms_dir, "launch")
    coug_fg_dir = get_package_share_directory("coug_fg")
    coug_fg_launch_dir = os.path.join(coug_fg_dir, "launch")
    coug_mapviz_dir = get_package_share_directory("coug_mapviz")
    coug_mapviz_launch_dir = os.path.join(coug_mapviz_dir, "launch")
    coug_rqt_dir = get_package_share_directory("coug_rqt")
    coug_rqt_launch_dir = os.path.join(coug_rqt_dir, "launch")

    coug_comms_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_comms_launch_dir, "coug_comms_base.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list_config,
            "scenario_param_file": scenario_param_file,
            "lead_agent": lead_agent,
            "enable_direct_comms": enable_direct_comms,
            "enable_acoustic_comms": enable_acoustic_comms,
        }.items(),
    )

    coug_fg_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(coug_fg_launch_dir, "coug_fg_base.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list_config,
            "scenario_param_file": scenario_param_file,
        }.items(),
    )

    coug_mapviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_mapviz_launch_dir, "coug_mapviz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list_config,
            "scenario_param_file": scenario_param_file,
            "initialize_origin": initialize_origin,
        }.items(),
    )

    coug_rqt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(coug_rqt_launch_dir, "coug_rqt.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list_config,
            "scenario_param_file": scenario_param_file,
        }.items(),
    )

    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        name="plotjuggler",
        arguments=[
            "-l",
            create_plotjuggler_config(agent_list),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", create_rviz_config(agent_list)],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    base_station_group = GroupAction(
        condition=IfCondition(enable_base_processing),
        actions=[
            PushRosNamespace("base_station"),
            coug_comms_base_launch,
            coug_fg_base_launch,
        ],
    )

    actions: list[Action] = [
        base_station_group,
        coug_mapviz_launch,
        coug_rqt_launch,
        plotjuggler_node,
        rviz_node,
    ]

    if record_bag_path_str:
        sim_time_args = ["--use-sim-time"] if IfCondition(use_sim_time).evaluate(context) else []
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
                *sim_time_args,
            ],
            sigterm_timeout="15",
            sigkill_timeout="15",
        )
        actions.append(record_process)

        atexit.register(save_artifacts, record_bag_path_str, snapshot_config())

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOG_DIR", launch_config.log_dir),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "agent_list",
                default_value="[auv0]",
            ),
            DeclareLaunchArgument(
                "scenario_param_file",
                default_value="",
            ),
            DeclareLaunchArgument(
                "lead_agent",
                default_value="",
            ),
            DeclareLaunchArgument(
                "record_bag_path",
                default_value="",
            ),
            DeclareLaunchArgument(
                "enable_direct_comms",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_acoustic_comms",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_base_processing",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "initialize_origin",
                default_value="true",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

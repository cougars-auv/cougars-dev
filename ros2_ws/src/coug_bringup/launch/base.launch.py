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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def save_artifacts(record_bag_path: str) -> None:
    if not record_bag_path or not os.path.isdir(record_bag_path):
        return

    artifacts = (
        ("Config", os.environ.get("CONFIG_DIR", ""), "config"),
        ("Logs", launch_config.log_dir, "log"),
    )
    for label, source, directory in artifacts:
        if os.path.isdir(source):
            destination = os.path.join(record_bag_path, directory)
            shutil.copytree(source, destination, dirs_exist_ok=True)
            print(f"{label} saved: {destination}")


def create_gui_config(template_name: str, agent_ns: str, suffix: str) -> str:
    config_dir = os.environ["CONFIG_DIR"]
    template_path = os.path.join(config_dir, "gui", template_name)
    with open(template_path) as template:
        content = template.read().replace("AGENT_NS", agent_ns)

    with tempfile.NamedTemporaryFile(
        mode="w", delete=False, suffix=suffix
    ) as rendered_config:
        rendered_config.write(content)
        return rendered_config.name


def create_rviz_config(agent_list: list[str]) -> str:
    if len(agent_list) == 1:
        return create_gui_config("rviz.rviz.template", agent_list[0], ".rviz")

    config_dir = os.environ["CONFIG_DIR"]
    gui_dir = os.path.join(config_dir, "gui")
    with open(os.path.join(gui_dir, "rviz.rviz.template")) as template:
        config = yaml.safe_load(template.read().replace("AGENT_NS", agent_list[0]))

    displays = config["Visualization Manager"]["Displays"]
    displays[:] = [
        display
        for display in displays
        if display.get("Class")
        in {"rviz_default_plugins/Grid", "rviz_default_plugins/TF"}
    ]
    with open(os.path.join(gui_dir, "multi_rviz.rviz.template")) as template:
        agent_template = template.read()
    displays.extend(
        display
        for agent_ns in agent_list
        for display in yaml.safe_load(agent_template.replace("AGENT_NS", agent_ns))[
            "displays"
        ]
    )
    with tempfile.NamedTemporaryFile(
        mode="w", delete=False, suffix=".rviz"
    ) as rendered_config:
        yaml.safe_dump(config, rendered_config, sort_keys=False)
        return rendered_config.name


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list_config = LaunchConfiguration("agent_list")
    lead_agent = LaunchConfiguration("lead_agent")
    enable_direct_comms = LaunchConfiguration("enable_direct_comms")
    enable_acoustic_comms = LaunchConfiguration("enable_acoustic_comms")
    record_bag_path = LaunchConfiguration("record_bag_path")

    agent_list_str = agent_list_config.perform(context)
    record_bag_path_str = record_bag_path.perform(context)

    agent_list = yaml.safe_load(agent_list_str)
    agent_ns = agent_list[0]

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
            "agent_list": agent_list_str,
            "lead_agent": lead_agent,
            "enable_direct_comms": enable_direct_comms,
            "enable_acoustic_comms": enable_acoustic_comms,
        }.items(),
    )

    coug_fg_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_fg_launch_dir, "coug_fg_base.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list_str,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", create_rviz_config(agent_list)],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        name="plotjuggler",
        arguments=[
            "-l",
            create_gui_config("plotjuggler.xml.template", agent_ns, ".xml"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    coug_mapviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_mapviz_launch_dir, "coug_mapviz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list_str,
        }.items(),
    )

    coug_rqt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_rqt_launch_dir, "coug_rqt.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list_str,
        }.items(),
    )

    base_station_group = GroupAction(
        actions=[
            PushRosNamespace("base_station"),
            coug_comms_base_launch,
            coug_fg_base_launch,
        ]
    )

    actions = [
        base_station_group,
        rviz_node,
        plotjuggler_node,
        coug_mapviz_launch,
        coug_rqt_launch,
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

        atexit.register(save_artifacts, record_bag_path_str)

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

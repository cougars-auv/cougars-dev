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

import os
import shutil
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_ros.actions import Node, PushRosNamespace


def save_config(context, record_bag_path) -> list:
    """
    Copy the active config directory into the recorded bag directory.

    :param record_bag_path: Path to the recorded bag (skipped if not created).
    """
    if not record_bag_path or not os.path.isdir(record_bag_path):
        return []

    config_dir = os.environ.get("CONFIG_DIR", "")
    if not config_dir or not os.path.isdir(config_dir):
        return []

    dest = os.path.join(record_bag_path, "config")
    shutil.copytree(config_dir, dest, dirs_exist_ok=True)
    return [LogInfo(msg=f"Config saved to {dest}")]


def launch_setup(context, *args, **kwargs) -> list:

    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list = LaunchConfiguration("agent_list")
    lead_agent = LaunchConfiguration("lead_agent")
    record_bag_path = LaunchConfiguration("record_bag_path")
    loc_comparison = LaunchConfiguration("loc_comparison")
    add_noise = LaunchConfiguration("add_noise")
    base_station = LaunchConfiguration("base_station")
    enable_direct_comms = LaunchConfiguration("enable_direct_comms")
    enable_acoustic_comms = LaunchConfiguration("enable_acoustic_comms")
    hitl_mode = LaunchConfiguration("hitl_mode")

    agent_list_str = agent_list.perform(context)
    record_bag_path_str = record_bag_path.perform(context)

    agent_namespaces = yaml.safe_load(agent_list_str)

    coug_bringup_dir = get_package_share_directory("coug_bringup")
    coug_bringup_launch_dir = os.path.join(coug_bringup_dir, "launch")
    holo_bridge_dir = get_package_share_directory("holoocean_bridge")
    holo_bridge_launch_dir = os.path.join(holo_bridge_dir, "launch")
    fleet_params = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "holoocean_bridge_params.yaml"]
    )

    actions = []

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
        )
        actions.append(record_process)

        actions.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=record_process,
                    on_exit=[
                        OpaqueFunction(
                            function=save_config,
                            kwargs={"record_bag_path": record_bag_path_str},
                        )
                    ],
                )
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(coug_bringup_launch_dir, "base.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "agent_list": agent_list_str,
                "lead_agent": lead_agent,
                "enable_direct_comms": enable_direct_comms,
                "enable_acoustic_comms": enable_acoustic_comms,
            }.items(),
        )
    )

    for auv_ns in agent_namespaces:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(coug_bringup_launch_dir, "auv.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "auv_ns": auv_ns,
                    "loc_comparison": loc_comparison,
                }.items(),
                condition=UnlessCondition(hitl_mode),
            )
        )

        bridge_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(holo_bridge_launch_dir, "holoocean_bridge.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "auv_ns": auv_ns,
                "add_noise": add_noise,
            }.items(),
        )

        actions.append(GroupAction(actions=[PushRosNamespace(auv_ns), bridge_launch]))

    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_holoocean_transform",
            arguments=[
                "--frame-id",
                "map",
                "--child-frame-id",
                "holoocean_global_frame",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    actions.append(
        GroupAction(
            actions=[
                PushRosNamespace("base_station"),
                Node(
                    package="holoocean_bridge",
                    executable="modem_converter",
                    name="modem_converter_node",
                    parameters=[
                        fleet_params,
                        {
                            "use_sim_time": use_sim_time,
                            "beacon_id": 15,
                            "modem_frame": "base_station",
                        },
                    ],
                ),
            ],
            condition=IfCondition(base_station),
        )
    )

    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_base_station_transform",
            arguments=[
                "--x",
                "-5",
                "--y",
                "0",
                "--z",
                "0",
                "--yaw",
                "0",
                "--pitch",
                "0",
                "--roll",
                "0",
                "--frame-id",
                "map",
                "--child-frame-id",
                "base_station",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(base_station),
        )
    )

    actions.append(
        Node(
            package="diagnostic_common_diagnostics",
            executable="cpu_monitor.py",
            name="cpu_monitor",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    actions.append(
        Node(
            package="diagnostic_common_diagnostics",
            executable="hd_monitor.py",
            name="hd_monitor",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    actions.append(
        Node(
            package="diagnostic_common_diagnostics",
            executable="ram_monitor.py",
            name="ram_monitor",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "agent_list",
                default_value="[coug1sim]",
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
                "loc_comparison",
                default_value="false",
                description="Launch additional localization nodes if true",
            ),
            DeclareLaunchArgument(
                "add_noise",
                default_value="true",
                description="Whether to add noise to sensor data",
            ),
            DeclareLaunchArgument(
                "base_station",
                default_value="true",
                description="Launch base station simulation nodes",
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
            DeclareLaunchArgument(
                "hitl_mode",
                default_value="false",
                description="HITL mode (skip launching local AUV nodes)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

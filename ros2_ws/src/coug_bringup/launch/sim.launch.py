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
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_ros.actions import Node, PushRosNamespace


def launch_setup(context, *args, **kwargs) -> list:

    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list = LaunchConfiguration("agent_list")
    record_bag_path = LaunchConfiguration("record_bag_path")
    compare = LaunchConfiguration("compare")
    add_noise = LaunchConfiguration("add_noise")
    base_station = LaunchConfiguration("base_station")

    agent_list_str = context.perform_substitution(agent_list)
    record_bag_path_str = context.perform_substitution(record_bag_path)

    agent_tuples = yaml.safe_load(agent_list_str)

    coug_bringup_dir = get_package_share_directory("coug_bringup")
    coug_bringup_launch_dir = os.path.join(coug_bringup_dir, "launch")
    holo_bridge_dir = get_package_share_directory("holoocean_bridge")
    holo_bridge_launch_dir = os.path.join(holo_bridge_dir, "launch")
    fleet_params = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "holoocean_bridge_params.yaml"]
    )

    actions = []

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
                    "--exclude-topics",
                    "/clock",
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
                "agent_list": agent_list_str,
            }.items(),
        )
    )

    for i, auv_ns in enumerate(agent_tuples):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(coug_bringup_launch_dir, "auv.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "auv_ns": auv_ns,
                    "compare": compare,
                }.items(),
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
                            "beacon_id": 0,
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
                    "YAML list of AUV namespaces "
                    "(e.g. '[coug1sim]' or '[coug1sim, coug2sim]')"
                ),
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
            DeclareLaunchArgument(
                "add_noise",
                default_value="true",
                description="Whether to add noise to sensor data",
            ),
            DeclareLaunchArgument(
                "base_station",
                default_value="true",
                description="Launch base station converters",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

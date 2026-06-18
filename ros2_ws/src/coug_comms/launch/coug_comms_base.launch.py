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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs) -> list:
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_direct_comms = LaunchConfiguration("enable_direct_comms")
    enable_acoustic_comms = LaunchConfiguration("enable_acoustic_comms")
    agent_list_str = LaunchConfiguration("agent_list").perform(context)

    agent_namespaces = yaml.safe_load(agent_list_str)

    config_dir = os.environ.get("CONFIG_DIR", "")

    fleet_default_beacon_id = None
    fleet_params_path = os.path.join(config_dir, "fleet", "coug_comms_params.yaml")
    if os.path.isfile(fleet_params_path):
        with open(fleet_params_path) as f:
            fleet_config = yaml.safe_load(f) or {}
        fleet_comms = (
            fleet_config.get("/**", {})
            .get("coug_comms_base_launch", {})
            .get("ros__parameters", {})
        )
        fleet_default_beacon_id = fleet_comms.get("beacon_id")

    beacon_ids = {}
    for ns in agent_namespaces:
        beacon_id = fleet_default_beacon_id
        agent_params_path = os.path.join(config_dir, f"{ns}_params.yaml")
        if os.path.isfile(agent_params_path):
            with open(agent_params_path) as f:
                agent_params = yaml.safe_load(f) or {}
            ns_params = agent_params.get(f"/{ns}", {})
            comms_params = ns_params.get("coug_comms_base_launch", {}).get(
                "ros__parameters", {}
            )
            beacon_id = comms_params.get("beacon_id", beacon_id)
        if beacon_id is not None:
            beacon_ids[ns] = beacon_id

    fleet_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "coug_comms_params.yaml",
        ]
    )

    return [
        Node(
            package="coug_comms",
            executable="base_dispatcher",
            name="base_dispatcher_node",
            namespace="base_station",
            parameters=[
                fleet_params,
                {
                    "agent_namespaces": agent_namespaces,
                    "beacon_ids": beacon_ids,
                    "use_sim_time": use_sim_time,
                    "enable_direct_comms": enable_direct_comms,
                    "enable_acoustic_comms": enable_acoustic_comms,
                },
            ],
        ),
        Node(
            package="coug_comms",
            executable="base_status_poller",
            name="base_status_poller_node",
            namespace="base_station",
            parameters=[
                fleet_params,
                {
                    "agent_namespaces": agent_namespaces,
                    "beacon_ids": beacon_ids,
                    "use_sim_time": use_sim_time,
                    "enable_direct_comms": enable_direct_comms,
                    "enable_acoustic_comms": enable_acoustic_comms,
                },
            ],
        ),
        Node(
            package="coug_comms",
            executable="base_status_extractor",
            name="base_status_extractor_node",
            namespace="base_station",
            parameters=[
                fleet_params,
                {
                    "agent_namespaces": agent_namespaces,
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "agent_list",
                default_value="[auv0]",
                description=(
                    "YAML list of AUV namespaces "
                    "(e.g. '[coug1sim]' or '[coug1sim, coug2sim]')"
                ),
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

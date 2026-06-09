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
    agent_list_str = LaunchConfiguration("agent_list").perform(context)

    agent_tuples = yaml.safe_load(agent_list_str)
    agent_namespaces = [t[0] for t in agent_tuples]

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
            executable="base_comms",
            name="base_comms_node",
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
                default_value="[[auv0, auv.urdf.xacro]]",
                description=(
                    "YAML list of [auv_ns, auv_urdf] pairs "
                    "(e.g. '[[coug1sim, couguv_holoocean.urdf.xacro], "
                    "[coug2sim, couguv_holoocean.urdf.xacro]]')"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

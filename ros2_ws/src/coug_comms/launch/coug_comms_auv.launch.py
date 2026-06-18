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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)


def generate_launch_description() -> LaunchDescription:

    use_sim_time = LaunchConfiguration("use_sim_time")
    auv_ns = LaunchConfiguration("auv_ns")

    fleet_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "coug_comms_params.yaml",
        ]
    )
    auv_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            PythonExpression(["'", auv_ns, "' + '_params.yaml'"]),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "auv_ns",
                default_value="auv0",
                description="Namespace for the AUV (e.g. auv0)",
            ),
            Node(
                package="coug_comms",
                executable="auv_receiver",
                name="auv_receiver_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="coug_comms",
                executable="auv_status_stager",
                name="auv_status_stager_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="coug_comms",
                executable="auv_status_bundler",
                name="auv_status_bundler_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )

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

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def agent_frame(agent_ns: str, frame: str) -> str:
    return f"{agent_ns}/{frame}" if agent_ns else frame


def load_launch_params(path: str, top_key: str) -> dict[str, Any]:
    try:
        with open(path) as config_file:
            config = yaml.safe_load(config_file)
        params = config[top_key]["coug_terrain_launch"]["ros__parameters"]
        return dict(params)
    except (KeyError, TypeError, OSError):
        return {}


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_ns = LaunchConfiguration("agent_ns")

    agent_ns_str = agent_ns.perform(context)

    config_dir = os.environ["CONFIG_DIR"]
    coug_terrain_dir = get_package_share_directory("coug_terrain")

    fleet_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_terrain_params.yaml"]
    )
    agent_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), [agent_ns, "_params.yaml"]]
    )
    scenario_param_file = (
        LaunchConfiguration("scenario_param_file").perform(context) or agent_param_file
    )

    fleet_launch_params = load_launch_params(
        os.path.join(config_dir, "fleet", "coug_terrain_params.yaml"), "/**"
    )
    agent_launch_params = load_launch_params(
        os.path.join(config_dir, f"{agent_ns_str}_params.yaml"), f"/{agent_ns_str}"
    )
    scenario_launch_params = load_launch_params(scenario_param_file, "/**")
    dem_filename = scenario_launch_params.get(
        "dem_file", agent_launch_params.get("dem_file", fleet_launch_params.get("dem_file"))
    )
    dem_file = os.path.join(coug_terrain_dir, "dem", dem_filename) if dem_filename else ""

    return [
        Node(
            package="coug_terrain",
            executable="dem_global_costmap",
            name="dem_global_costmap_node",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "dem_file": dem_file,
                    "map_frame": "map",
                },
            ],
        ),
        Node(
            package="ground_segmentation_ros2",
            executable="ground_segmentation_ros2_node",
            name="ground_segmentation_node",
            additional_env={"PCL_VERBOSITY_LEVEL": "ALWAYS"},
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {
                    "use_sim_time": use_sim_time,
                    "robot_frame": agent_frame(agent_ns_str, "base_link"),
                },
            ],
            remappings=[
                ("/ground_segmentation/input_pointcloud", "camera/point_cloud/cloud_registered"),
                ("/ground_segmentation/input_imu", "camera/imu/data"),
                ("/ground_segmentation/ground_points", "ground_segmentation/ground_points"),
                ("/ground_segmentation/obstacle_points", "ground_segmentation/obstacle_points"),
                ("/ground_segmentation/raw_points", "ground_segmentation/raw_points"),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "agent_ns",
                default_value="auv0",
            ),
            DeclareLaunchArgument(
                "scenario_param_file",
                default_value="",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

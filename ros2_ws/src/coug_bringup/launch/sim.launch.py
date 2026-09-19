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

import json
import math
import os
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace


def as_meters(position: list[float]) -> list[float]:
    return [float(value) for value in position]


def as_radians(orientation: list[float]) -> list[float]:
    return [math.radians(float(value)) for value in orientation]


def load_launch_params(path: str, top_key: str) -> dict[str, Any]:
    try:
        with open(path) as config_file:
            config = yaml.safe_load(config_file)
        params = config[top_key]["sim_launch"]["ros__parameters"]
        return dict(params)
    except (KeyError, TypeError, OSError):
        return {}


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    scenario_param_file = LaunchConfiguration("scenario_param_file")
    record_bag_path = LaunchConfiguration("record_bag_path")
    add_noise = LaunchConfiguration("add_noise")
    loc_comparison = LaunchConfiguration("loc_comparison")
    lead_agent = LaunchConfiguration("lead_agent")
    enable_direct_comms = LaunchConfiguration("enable_direct_comms")
    enable_acoustic_comms = LaunchConfiguration("enable_acoustic_comms")
    known_initial_poses = LaunchConfiguration("known_initial_poses").perform(context) == "true"
    enable_mapping = LaunchConfiguration("enable_mapping")
    hitl_mode = LaunchConfiguration("hitl_mode")

    scenario_param_file_str = scenario_param_file.perform(context)

    config_dir = os.environ["CONFIG_DIR"]
    fleet_launch_params = load_launch_params(
        os.path.join(config_dir, "fleet", "coug_bringup_params.yaml"), "/**"
    )
    scenario_launch_params = load_launch_params(scenario_param_file_str, "/**")
    use_gazebo = os.path.basename(os.path.dirname(scenario_param_file_str)) == "gazebo"

    base_station: dict[str, Any] = {}
    if use_gazebo:
        pose_source = scenario_param_file_str
        agent_poses = {
            agent_ns: load_launch_params(scenario_param_file_str, f"/{agent_ns}")
            for agent_ns in scenario_launch_params.get("agents", [])
        }
    else:
        scenario_filename = scenario_launch_params.get(
            "scenario_file", fleet_launch_params.get("scenario_file")
        )
        scenario_file = os.path.join(config_dir, "holoocean", scenario_filename)
        pose_source = scenario_file
        with open(scenario_file) as scenario_config:
            agents = json.load(scenario_config)["agents"]
        agent_poses = {agent["agent_name"]: agent for agent in agents}
        base_station = agent_poses.pop("base_station")

    agent_list = list(agent_poses)
    agent_list_str = f"[{', '.join(agent_list)}]"

    coug_bringup_dir = get_package_share_directory("coug_bringup")
    coug_bringup_launch_dir = os.path.join(coug_bringup_dir, "launch")
    coug_gazebo_dir = get_package_share_directory("coug_gazebo")
    coug_gazebo_launch_dir = os.path.join(coug_gazebo_dir, "launch")
    coug_holoocean_dir = get_package_share_directory("coug_holoocean")
    coug_holoocean_launch_dir = os.path.join(coug_holoocean_dir, "launch")
    fleet_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_bringup_params.yaml"]
    )
    holoocean_fleet_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_holoocean_params.yaml"]
    )

    actions: list[Action] = []

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(coug_bringup_launch_dir, "base.launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "agent_list": agent_list_str,
                "scenario_param_file": scenario_param_file,
                "lead_agent": lead_agent,
                "record_bag_path": record_bag_path,
                "enable_direct_comms": enable_direct_comms,
                "enable_acoustic_comms": enable_acoustic_comms,
            }.items(),
        )
    )

    if use_gazebo:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(coug_gazebo_launch_dir, "coug_gazebo_world.launch.py")
                ),
                launch_arguments={
                    "scenario_param_file": scenario_param_file,
                }.items(),
            )
        )

    for agent_ns in agent_list:
        agent_pose = agent_poses[agent_ns]
        if "location" not in agent_pose or "rotation" not in agent_pose:
            raise RuntimeError(f"No 'location' and 'rotation' set for {agent_ns} in {pose_source}")
        position = str(as_meters(agent_pose["location"]))
        orientation = str(as_radians(agent_pose["rotation"]))

        agent_param_file = PathJoinSubstitution(
            [EnvironmentVariable("CONFIG_DIR"), f"{agent_ns}_params.yaml"]
        )

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(coug_bringup_launch_dir, "agent.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "agent_ns": agent_ns,
                    "scenario_param_file": scenario_param_file,
                    "loc_comparison": loc_comparison,
                    "lead_agent": lead_agent,
                    "initial_position": position if known_initial_poses else "[0.0, 0.0, 0.0]",
                    "initial_orientation": (
                        orientation if known_initial_poses else "[0.0, 0.0, 0.0]"
                    ),
                }.items(),
                condition=UnlessCondition(hitl_mode),
            )
        )

        if use_gazebo:
            sim_bridge_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(coug_gazebo_launch_dir, "coug_gazebo_agent.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "agent_ns": agent_ns,
                    "scenario_param_file": scenario_param_file,
                    "initial_position": position,
                    "initial_orientation": orientation,
                }.items(),
            )
        else:
            sim_bridge_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(coug_holoocean_launch_dir, "coug_holoocean.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "agent_ns": agent_ns,
                    "scenario_param_file": scenario_param_file,
                    "add_noise": add_noise,
                }.items(),
            )

        actions.append(GroupAction(actions=[PushRosNamespace(agent_ns), sim_bridge_launch]))

        actions.append(
            Node(
                package="voxblox_ros",
                executable="tsdf_server",
                name="voxblox_node",
                namespace=agent_ns,
                remappings=[
                    ("pointcloud_1", "camera/point_cloud/cloud_registered"),
                ],
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    scenario_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "world_frame": "map",
                    },
                ],
                condition=IfCondition(enable_mapping),
            )
        )

        if agent_ns in ("rover1sim", "rover2sim"):
            actions.append(
                Node(
                    package="ground_segmentation_ros2",
                    executable="ground_segmentation_ros2_node",
                    name="ground_segmentation_node",
                    namespace=agent_ns,
                    parameters=[
                        fleet_param_file,
                        agent_param_file,
                        scenario_param_file,
                        {
                            "use_sim_time": use_sim_time,
                            "robot_frame": f"{agent_ns}/base_link",
                        },
                    ],
                    remappings=[
                        (
                            "/ground_segmentation/input_pointcloud",
                            "camera/point_cloud/cloud_registered",
                        ),
                        ("/ground_segmentation/input_imu", "camera/imu/data"),
                        (
                            "/ground_segmentation/ground_points",
                            "ground_segmentation/ground_points",
                        ),
                        (
                            "/ground_segmentation/obstacle_points",
                            "ground_segmentation/obstacle_points",
                        ),
                        ("/ground_segmentation/raw_points", "ground_segmentation/raw_points"),
                    ],
                )
            )

    if not use_gazebo:
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
                        package="coug_holoocean",
                        executable="depth_converter",
                        name="modem_depth_converter_node",
                        parameters=[
                            holoocean_fleet_param_file,
                            scenario_param_file,
                            {
                                "use_sim_time": use_sim_time,
                                "add_noise": add_noise,
                                "depth_frame": "base_station",
                                "map_frame": "map",
                            },
                        ],
                    ),
                    Node(
                        package="coug_holoocean",
                        executable="modem_converter",
                        name="modem_converter_node",
                        parameters=[
                            holoocean_fleet_param_file,
                            scenario_param_file,
                            {
                                "use_sim_time": use_sim_time,
                                "add_noise": add_noise,
                                "modem_frame": "base_station",
                            },
                        ],
                    ),
                ],
            )
        )

        if "location" not in base_station or "rotation" not in base_station:
            raise RuntimeError(
                f"No 'location' and 'rotation' set for base_station in {pose_source}"
            )
        base_station_position = as_meters(base_station["location"])
        base_station_orientation = as_radians(base_station["rotation"])
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_base_station_transform",
                arguments=[
                    "--x",
                    str(base_station_position[0]),
                    "--y",
                    str(base_station_position[1]),
                    "--z",
                    str(base_station_position[2]),
                    "--roll",
                    str(base_station_orientation[0]),
                    "--pitch",
                    str(base_station_orientation[1]),
                    "--yaw",
                    str(base_station_orientation[2]),
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "base_station",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
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
                "scenario_param_file",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("CONFIG_DIR"), "holoocean", "couguv_openwater_params.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "record_bag_path",
                default_value="",
            ),
            DeclareLaunchArgument(
                "add_noise",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "loc_comparison",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "lead_agent",
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
                "known_initial_poses",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_mapping",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "hitl_mode",
                default_value="false",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

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
    OrSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list_config = LaunchConfiguration("agent_list")
    lead_agent = LaunchConfiguration("lead_agent")
    record_bag_path = LaunchConfiguration("record_bag_path")
    loc_comparison = LaunchConfiguration("loc_comparison")
    add_noise = LaunchConfiguration("add_noise")
    enable_mapping = LaunchConfiguration("enable_mapping")
    enable_shared_mapping = LaunchConfiguration("enable_shared_mapping")
    enable_direct_comms = LaunchConfiguration("enable_direct_comms")
    enable_acoustic_comms = LaunchConfiguration("enable_acoustic_comms")
    hitl_mode = LaunchConfiguration("hitl_mode")

    agent_list_str = agent_list_config.perform(context)

    agent_list = yaml.safe_load(agent_list_str)

    coug_bringup_dir = get_package_share_directory("coug_bringup")
    coug_bringup_launch_dir = os.path.join(coug_bringup_dir, "launch")
    coug_holo_dir = get_package_share_directory("coug_holoocean")
    coug_holo_launch_dir = os.path.join(coug_holo_dir, "launch")
    fleet_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_holoocean_params.yaml"]
    )

    actions = []

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(coug_bringup_launch_dir, "base.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "agent_list": agent_list_str,
                "lead_agent": lead_agent,
                "record_bag_path": record_bag_path,
                "enable_direct_comms": enable_direct_comms,
                "enable_acoustic_comms": enable_acoustic_comms,
            }.items(),
        )
    )

    for agent_ns in agent_list:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(coug_bringup_launch_dir, "agent.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "agent_ns": agent_ns,
                    "lead_agent": lead_agent,
                    "loc_comparison": loc_comparison,
                }.items(),
                condition=UnlessCondition(hitl_mode),
            )
        )

        bridge_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(coug_holo_launch_dir, "coug_holoocean.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "agent_ns": agent_ns,
                "add_noise": add_noise,
            }.items(),
        )

        actions.append(GroupAction(actions=[PushRosNamespace(agent_ns), bridge_launch]))

        actions.append(
            GroupAction(
                condition=IfCondition(
                    OrSubstitution(enable_mapping, enable_shared_mapping)
                ),
                actions=[
                    PushRosNamespace(agent_ns),
                    ComposableNodeContainer(
                        package="rclcpp_components",
                        executable="component_container",
                        name="depth_camera_container",
                        namespace="",
                        composable_node_descriptions=[
                            ComposableNode(
                                package="depth_image_proc",
                                plugin="depth_image_proc::PointCloudXyzrgbNode",
                                name="depth_camera_cloud_node",
                                remappings=[
                                    ("depth_registered/image_rect", "depth/image_rect"),
                                    ("rgb/image_rect_color", "depth/image_rect_color"),
                                    ("points", "depth/points"),
                                ],
                                parameters=[{"use_sim_time": use_sim_time}],
                            ),
                        ],
                    ),
                    Node(
                        package="voxblox_ros",
                        executable="tsdf_server",
                        name="voxblox_node",
                        condition=IfCondition(enable_mapping),
                        remappings=[
                            ("pointcloud_1", "depth/points"),
                        ],
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "world_frame": "map",
                                "tsdf_voxel_size": 0.05,
                                "method": "fast",
                            }
                        ],
                    ),
                ],
            )
        )

    actions.append(
        Node(
            package="voxblox_ros",
            executable="tsdf_server",
            name="shared_voxblox_node",
            condition=IfCondition(enable_shared_mapping),
            remappings=[
                (f"pointcloud_{index}", f"/{agent_ns}/depth/points")
                for index, agent_ns in enumerate(agent_list, start=1)
            ],
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "world_frame": "map",
                    "num_pointcloud_subs": len(agent_list),
                    "tsdf_voxel_size": 0.05,
                    "method": "fast",
                }
            ],
        )
    )

    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_holoocean_transform",
            # Match this to the starting XY location of the first agent
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
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
                        fleet_param_file,
                        {
                            "use_sim_time": use_sim_time,
                            "depth_frame": "base_station",
                            "map_frame": "map",
                            "add_noise": add_noise,
                        },
                    ],
                ),
                Node(
                    package="coug_holoocean",
                    executable="modem_converter",
                    name="modem_converter_node",
                    parameters=[
                        fleet_param_file,
                        {
                            "use_sim_time": use_sim_time,
                            "beacon_id": 15,
                            "modem_frame": "base_station",
                            "add_noise": add_noise,
                        },
                    ],
                ),
            ],
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
            SetEnvironmentVariable("ROS_LOG_DIR", launch_config.log_dir),
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
                "enable_mapping",
                default_value="false",
                description="Launch the depth camera and voxblox mapping pipeline",
            ),
            DeclareLaunchArgument(
                "enable_shared_mapping",
                default_value="false",
                description="Fuse all agent depth clouds into one Voxblox mesh",
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

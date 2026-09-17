# Copyright (c) 2018 Intel Corporation
# Copyright (C) 2024 Stevedan Ogochukwu Omodolor Omodia
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
import tempfile
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer


def load_launch_params(path: str, top_key: str) -> dict[str, Any]:
    try:
        with open(path) as config_file:
            config = yaml.safe_load(config_file)
        params = config[top_key]["coug_gazebo_world_launch"]["ros__parameters"]
        return dict(params)
    except (KeyError, TypeError, OSError):
        return {}


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    scenario_param_file = LaunchConfiguration("scenario_param_file")
    headless = LaunchConfiguration("headless")

    config_dir = os.environ["CONFIG_DIR"]
    coug_gazebo_dir = get_package_share_directory("coug_gazebo")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    ros_gz_sim_launch_dir = os.path.join(ros_gz_sim_dir, "launch")

    clock_bridge_config_file = os.path.join(coug_gazebo_dir, "config", "clock_bridge.yaml")

    fleet_launch_params = load_launch_params(
        os.path.join(config_dir, "fleet", "coug_gazebo_params.yaml"), "/**"
    )
    scenario_launch_params = load_launch_params(scenario_param_file.perform(context), "/**")
    world_filename = scenario_launch_params.get("world_file", fleet_launch_params.get("world_file"))
    world_file = os.path.join(coug_gazebo_dir, "worlds", world_filename)
    world_sdf_file = tempfile.mktemp(prefix="coug_gazebo_", suffix=".sdf")

    world_xacro_process = ExecuteProcess(
        cmd=["xacro", "-o", world_sdf_file, ["headless:=", headless], world_file],
    )

    return [
        world_xacro_process,
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf_file))]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=world_xacro_process,
                on_exit=[
                    GzServer(
                        world_sdf_file=world_sdf_file,
                        container_name="/gazebo_container",
                        use_composition=True,
                        verbosity_level=1,
                    ),
                    ComposableNodeContainer(
                        package="rclcpp_components",
                        executable="component_container_isolated",
                        name="gazebo_container",
                        namespace="",
                    ),
                    RosGzBridge(
                        bridge_name="clock_bridge",
                        config_file=clock_bridge_config_file,
                        container_name="/gazebo_container",
                        use_composition=True,
                    ),
                ],
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_launch_dir, "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": "-v4 -g",
            }.items(),
            condition=UnlessCondition(headless),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario_param_file",
                default_value=PathJoinSubstitution(
                    [
                        EnvironmentVariable("CONFIG_DIR"),
                        "gazebo",
                        "rover_sonoma_raceway_params.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

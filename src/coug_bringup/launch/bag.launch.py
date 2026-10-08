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

import atexit
import os
import shutil
import signal
import tempfile
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events.process import SignalProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger, launch_config
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace


def snapshot_config() -> str:
    config_dir = os.environ.get("CONFIG_DIR", "")
    if not os.path.isdir(config_dir):
        return ""
    snapshot = tempfile.mkdtemp(prefix="config_")
    shutil.copytree(config_dir, snapshot, dirs_exist_ok=True)
    return snapshot


def save_artifacts(record_bag_path: str, config_snapshot: str) -> None:
    if not record_bag_path or not os.path.isdir(record_bag_path):
        return

    artifacts = (
        ("Config", config_snapshot, "config"),
        ("Logs", launch_config.log_dir, "log"),
    )
    for label, source, directory in artifacts:
        if os.path.isdir(source):
            destination = os.path.join(record_bag_path, directory)
            shutil.copytree(source, destination, dirs_exist_ok=True)
            get_logger("launch.user").info(f"{label} saved: {destination}")


def load_launch_params(path: str, top_key: str) -> dict[str, Any]:
    try:
        with open(path) as config_file:
            config = yaml.safe_load(config_file)
        params = config[top_key]["bag_launch"]["ros__parameters"]
        return dict(params)
    except (KeyError, TypeError, OSError):
        return {}


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list_config = LaunchConfiguration("agent_list")
    play_bag_path = LaunchConfiguration("play_bag_path")
    record_bag_path = LaunchConfiguration("record_bag_path")
    start_offset = LaunchConfiguration("start_offset")
    playback_duration = LaunchConfiguration("playback_duration")
    playback_rate = LaunchConfiguration("playback_rate")
    start_paused = LaunchConfiguration("start_paused")
    loc_comparison = LaunchConfiguration("loc_comparison")
    lead_agent = LaunchConfiguration("lead_agent")
    enable_mapping = LaunchConfiguration("enable_mapping")
    hitl_mode = LaunchConfiguration("hitl_mode")

    agent_list_str = agent_list_config.perform(context)
    play_bag_path_str = play_bag_path.perform(context)
    record_bag_path_str = record_bag_path.perform(context)

    agent_list = yaml.safe_load(agent_list_str)

    config_dir = os.environ["CONFIG_DIR"]
    coug_bringup_dir = get_package_share_directory("coug_bringup")
    coug_bringup_launch_dir = os.path.join(coug_bringup_dir, "launch")

    fleet_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_bringup_params.yaml"]
    )

    fleet_param_path = os.path.join(config_dir, "fleet", "coug_bringup_params.yaml")

    actions: list[Action] = []

    record_process = None

    if record_bag_path_str:
        config_snapshot = snapshot_config()
        sim_time_args = ["--use-sim-time"] if IfCondition(use_sim_time).evaluate(context) else []
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
                *sim_time_args,
            ],
            sigterm_timeout="15",
            sigkill_timeout="15",
        )
        actions.append(record_process)

        actions.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=record_process,
                    on_exit=[
                        OpaqueFunction(
                            function=lambda _: save_artifacts(record_bag_path_str, config_snapshot)
                        )
                    ],
                )
            )
        )

        atexit.register(save_artifacts, record_bag_path_str, config_snapshot)

    if play_bag_path_str:
        root_outputs = [
            "tf",
            "tf_static",
            "diagnostics.*",
            "origin",
            "local_xy_origin",
        ]
        agent_outputs = [
            "robot_description",
            "agent/status",
            "base/.*",
            "cmd_hsd.*",
            "led/color",
            "modem_send",
            "waypoints.*",
            "odometry/(local|global).*",
            "smoothed_path.*",
            "factor_graph_node.*",
            "imu/data_madgwick",
            "gps/odometry",
            "depth/odometry",
            "imu/mag_tesla",
            "seatrac/imu/data",
            "seatrac/depth/odometry",
            "dvl/odometry",
            "dvl/twist.*",
            "dvl/beams.*",
            r"dvl/beam\d/range",
        ]
        exclude_regex = "|".join(
            [f"^/({'|'.join(root_outputs)})$", f"^/[^/]+/({'|'.join(agent_outputs)})$"]
        )

        start_paused_args = (
            ["--start-paused"] if IfCondition(start_paused).evaluate(context) else []
        )
        play_process = ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                play_bag_path_str,
                "--clock",
                "--start-offset",
                start_offset,
                "--playback-duration",
                playback_duration,
                "--rate",
                playback_rate,
                *start_paused_args,
                "--exclude-regex",
                exclude_regex,
            ],
        )
        actions.append(play_process)

        exit_event: list[Action] = [LogInfo(msg="Bag playback finished; no recording to stop.")]
        if record_process is not None:
            exit_event = [
                LogInfo(msg="Bag playback finished; stopping recording."),
                EmitEvent(
                    event=SignalProcess(
                        signal_number=signal.SIGINT,
                        process_matcher=matches_action(record_process),
                    )
                ),
            ]

        actions.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=play_process,
                    on_exit=exit_event,
                )
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(coug_bringup_launch_dir, "base.launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "agent_list": agent_list_config,
                "lead_agent": lead_agent,
                "record_bag_path": "",
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
                    "loc_comparison": loc_comparison,
                    "lead_agent": lead_agent,
                }.items(),
                condition=UnlessCondition(hitl_mode),
            )
        )

        agent_param_file = PathJoinSubstitution(
            [EnvironmentVariable("CONFIG_DIR"), f"{agent_ns}_params.yaml"]
        )

        agent_param_path = os.path.join(config_dir, f"{agent_ns}_params.yaml")
        launch_params = {
            **load_launch_params(fleet_param_path, "/**"),
            **load_launch_params(agent_param_path, f"/{agent_ns}"),
        }
        pointcloud_topic = launch_params["pointcloud_topic"]

        actions.append(
            GroupAction(
                actions=[
                    PushRosNamespace(agent_ns),
                    Node(
                        package="voxblox_ros",
                        executable="tsdf_server",
                        name="tsdf_server",
                        condition=IfCondition(enable_mapping),
                        parameters=[
                            fleet_param_file,
                            agent_param_file,
                            {
                                "use_sim_time": use_sim_time,
                                "world_frame": "map",
                            },
                        ],
                        remappings=[("pointcloud_1", pointcloud_topic)],
                    ),
                ]
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
                "agent_list",
                default_value="[auv0]",
            ),
            DeclareLaunchArgument(
                "play_bag_path",
                default_value="",
            ),
            DeclareLaunchArgument(
                "record_bag_path",
                default_value="",
            ),
            DeclareLaunchArgument(
                "start_offset",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "playback_duration",
                default_value="-1.0",
            ),
            DeclareLaunchArgument(
                "playback_rate",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "start_paused",
                default_value="false",
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

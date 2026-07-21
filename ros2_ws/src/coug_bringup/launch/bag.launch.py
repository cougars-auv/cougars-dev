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

import atexit
import os
import shutil
import signal
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    EmitEvent,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events.process import SignalProcess
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration


def save_config(record_bag_path) -> None:
    """
    Copy the active config directory into the recorded bag directory.

    :param record_bag_path: Path to the recorded bag (skipped if not created).
    """
    if not record_bag_path or not os.path.isdir(record_bag_path):
        return

    config_dir = os.environ.get("CONFIG_DIR", "")
    if not config_dir or not os.path.isdir(config_dir):
        return

    dest = os.path.join(record_bag_path, "config")
    shutil.copytree(config_dir, dest, dirs_exist_ok=True)
    print(f"Config saved: {dest}")


def save_logs(record_bag_path) -> None:
    """
    Copy this run's ROS log directory into the recorded bag directory.

    :param record_bag_path: Path to the recorded bag (skipped if not created).
    """
    if not record_bag_path or not os.path.isdir(record_bag_path):
        return

    log_dir = launch_config.log_dir
    if not os.path.isdir(log_dir):
        return

    dest = os.path.join(record_bag_path, "log")
    shutil.copytree(log_dir, dest, dirs_exist_ok=True)
    print(f"Logs saved: {dest}")


def save_artifacts(context, record_bag_path) -> None:
    """
    Copy config and logs into the recorded bag as soon as recording stops.
    :param record_bag_path: Path to the recorded bag (skipped if not created).
    """
    save_config(record_bag_path)
    save_logs(record_bag_path)


def launch_setup(context, *args, **kwargs) -> list:
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_delay = LaunchConfiguration("start_delay")
    playback_rate = LaunchConfiguration("playback_rate")
    agent_list = LaunchConfiguration("agent_list")
    play_bag_path = LaunchConfiguration("play_bag_path")
    record_bag_path = LaunchConfiguration("record_bag_path")
    loc_comparison = LaunchConfiguration("loc_comparison")
    hitl_mode = LaunchConfiguration("hitl_mode")

    agent_list_str = agent_list.perform(context)
    play_bag_path_str = play_bag_path.perform(context)
    record_bag_path_str = record_bag_path.perform(context)

    agent_namespaces = yaml.safe_load(agent_list_str)
    auv_ns = agent_namespaces[0]

    coug_bringup_dir = get_package_share_directory("coug_bringup")
    coug_bringup_launch_dir = os.path.join(coug_bringup_dir, "launch")

    actions = []

    record_process = None

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
                            function=save_artifacts,
                            kwargs={"record_bag_path": record_bag_path_str},
                        )
                    ],
                )
            )
        )

        atexit.register(save_config, record_bag_path_str)
        atexit.register(save_logs, record_bag_path_str)

    if play_bag_path_str:
        play_process = ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                play_bag_path_str,
                "--clock",
                "--rate",
                playback_rate,
                "--start-offset",
                start_delay,
                "--remap",
                # FROST Lab BlueROV2 bags
                "/tf:=/tf_discard",
                "/tf_static:=/tf_static_discard",
                "/diagnostics_agg:=/diagnostics_agg_discard",
                "/origin:=/origin_discard",
                f"{auv_ns}/odometry/global:=/{auv_ns}/odometry/global_discard",
                f"{auv_ns}/dvl/twist:=/{auv_ns}/dvl/twist_discard",
                f"{auv_ns}/dvl/odometry:=/{auv_ns}/dvl/odometry_discard",
                f"{auv_ns}/imu/nav_sat_fix:=/{auv_ns}/gps/fix",
                f"{auv_ns}/shallow/pressure/data:=/{auv_ns}/pressure/data",
                # FROST Lab CougUV bags
                f"/pressure/data:=/{auv_ns}/pressure/data",
                f"/fix:=/{auv_ns}/gps/fix",
                f"/dvl/position:=/{auv_ns}/dvl/position",
                f"/dvl/data:=/{auv_ns}/dvl/data",
                f"/modem_status:=/{auv_ns}/modem_status",
                # TURTLMap bags
                f"/nav/filtered_imu/data:=/{auv_ns}/imu/data_ned",
                f"/BlueROV/pressure2_fluid:=/{auv_ns}/pressure/data",
            ],
        )
        actions.append(play_process)

        exit_event = [LogInfo(msg="Bag playback finished, no recording to kill.")]
        if record_process is not None:
            exit_event = [
                LogInfo(msg="Bag playback finished, killing recording."),
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
            PythonLaunchDescriptionSource(
                os.path.join(coug_bringup_launch_dir, "base.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "agent_list": agent_list_str,
                "record_bag_path": "",
            }.items(),
        )
    )

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
                "start_delay",
                default_value="0.0",
                description=(
                    "Time in seconds to skip from the beginning of the bag file (start offset)"
                ),
            ),
            DeclareLaunchArgument(
                "agent_list",
                default_value="[auv0]",
                description=(
                    "YAML list of agent namespaces "
                    "(e.g. '[coug1sim]' or '[coug1sim, coug2sim]')"
                ),
            ),
            DeclareLaunchArgument(
                "play_bag_path",
                default_value="",
                description="Path to play rosbag from",
            ),
            DeclareLaunchArgument(
                "record_bag_path",
                default_value="",
                description="Path to record rosbag (if empty, no recording)",
            ),
            DeclareLaunchArgument(
                "playback_rate",
                default_value="1.0",
                description="Bag playback rate multiplier (e.g. 0.5 for half speed, 2.0 for double)",
            ),
            DeclareLaunchArgument(
                "loc_comparison",
                default_value="false",
                description="Launch additional localization nodes if true",
            ),
            DeclareLaunchArgument(
                "hitl_mode",
                default_value="false",
                description="HITL mode (skip launching local AUV nodes)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

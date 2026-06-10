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
)
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events.process import SignalProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs) -> list:

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_delay = LaunchConfiguration("start_delay")
    playback_rate = LaunchConfiguration("playback_rate")
    agent_list = LaunchConfiguration("agent_list")
    play_bag_path = LaunchConfiguration("play_bag_path")
    record_bag_path = LaunchConfiguration("record_bag_path")
    compare = LaunchConfiguration("compare")

    agent_list_str = context.perform_substitution(agent_list)
    play_bag_path_str = context.perform_substitution(play_bag_path)
    record_bag_path_str = context.perform_substitution(record_bag_path)

    [(auv_ns, auv_urdf)] = yaml.safe_load(agent_list_str)

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
                "/tf:=/tf_discard",
                "/tf_static:=/tf_static_discard",
                # FROST Lab BlueROV2 bags
                f"{auv_ns}/dvl/twist:=/{auv_ns}/dvl/twist_discard",
                f"{auv_ns}/imu/nav_sat_fix:=/{auv_ns}/gps/fix",
                f"{auv_ns}/shallow/pressure/data:=/{auv_ns}/pressure",
                # FROST Lab CougUV bags
                f"/pressure/data:=/{auv_ns}/pressure",
                f"/fix:=/{auv_ns}/gps/fix",
                f"/dvl/position:=/{auv_ns}/dvl/position",
                f"/modem_status:=/{auv_ns}/modem_status",
                # TURTLMap bags
                f"/dvl/data:=/{auv_ns}/dvl/data",
                f"/nav/filtered_imu/data:=/{auv_ns}/imu/data_ned",
                f"/BlueROV/pressure2_fluid:=/{auv_ns}/pressure",
                # AQUA-SLAM bags
                f"/apriltag_slam/GT:=/{auv_ns}/odometry/truth",
                f"/aqua_slam/pose:=/{auv_ns}/odometry/global_aqs",
                f"/imu/data:=/{auv_ns}/imu/data_ned",
                f"/depth/data:=/{auv_ns}/odometry/depth",
            ],
        )
        actions.append(play_process)

        exit_event = LogInfo(msg="Bag playback finished, no recording to kill...")
        if record_process is not None:
            exit_event = [
                LogInfo(msg="Bag playback finished, killing recording..."),
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
                "auv_urdf": auv_urdf,
                "auv_ns": auv_ns,
                "compare": compare,
            }.items(),
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
                "start_delay",
                default_value="0.0",
                description=(
                    "Time in seconds to skip from the beginning of the bag file (start offset)"
                ),
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
                "compare",
                default_value="false",
                description="Launch additional localization nodes if true",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

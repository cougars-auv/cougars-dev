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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs) -> list:

    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_list = LaunchConfiguration("agent_list")
    lead_agent = LaunchConfiguration("lead_agent")
    enable_direct_comms = LaunchConfiguration("enable_direct_comms")
    enable_acoustic_comms = LaunchConfiguration("enable_acoustic_comms")
    agent_list_str = agent_list.perform(context)
    agent_namespaces = yaml.safe_load(agent_list_str)
    auv_ns = agent_namespaces[0]

    coug_comms_dir = get_package_share_directory("coug_comms")
    coug_comms_launch_dir = os.path.join(coug_comms_dir, "launch")
    coug_fgo_dir = get_package_share_directory("coug_fgo")
    coug_fgo_launch_dir = os.path.join(coug_fgo_dir, "launch")
    coug_active_fgo_dir = get_package_share_directory("coug_active_fgo")
    coug_active_fgo_launch_dir = os.path.join(coug_active_fgo_dir, "launch")
    coug_visual_dvl_dir = get_package_share_directory("coug_visual_dvl")
    coug_visual_dvl_launch_dir = os.path.join(coug_visual_dvl_dir, "launch")
    coug_mapviz_dir = get_package_share_directory("coug_mapviz")
    coug_mapviz_launch_dir = os.path.join(coug_mapviz_dir, "launch")
    coug_rqt_dir = get_package_share_directory("coug_rqt")
    coug_rqt_launch_dir = os.path.join(coug_rqt_dir, "launch")

    coug_comms_base_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_comms_launch_dir, "coug_comms_base.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list,
            "lead_agent": lead_agent,
            "enable_direct_comms": enable_direct_comms,
            "enable_acoustic_comms": enable_acoustic_comms,
        }.items(),
    )

    coug_fgo_base_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_fgo_launch_dir, "coug_fgo_base.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list,
        }.items(),
    )

    coug_fgo_viz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_fgo_launch_dir, "coug_fgo_viz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list,
            "launch_rviz": "true",
            "launch_plotjuggler": "true",
        }.items(),
    )

    coug_active_fgo_viz_cmd = IncludeLaunchDescription(  # noqa: F841
        PythonLaunchDescriptionSource(
            os.path.join(coug_active_fgo_launch_dir, "coug_active_fgo_viz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "auv_ns": auv_ns,
            "launch_rviz": "true",
            "launch_plotjuggler": "true",
        }.items(),
    )

    coug_visual_dvl_viz_cmd = IncludeLaunchDescription(  # noqa: F841
        PythonLaunchDescriptionSource(
            os.path.join(coug_visual_dvl_launch_dir, "coug_visual_dvl_viz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "auv_ns": auv_ns,
        }.items(),
    )

    coug_mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_mapviz_launch_dir, "coug_mapviz.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list,
        }.items(),
    )

    coug_rqt_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_rqt_launch_dir, "coug_rqt.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_list": agent_list,
        }.items(),
    )

    return [
        coug_comms_base_cmd,
        coug_fgo_base_cmd,
        coug_fgo_viz_cmd,
        # coug_active_fgo_viz_cmd,
        # coug_visual_dvl_viz_cmd,
        coug_mapviz_cmd,
        coug_rqt_cmd,
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

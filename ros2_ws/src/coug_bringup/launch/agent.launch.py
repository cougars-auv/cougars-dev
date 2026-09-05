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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.substitutions import (
    EnvironmentVariable,
    EqualsSubstitution,
    LaunchConfiguration,
    NotEqualsSubstitution,
    NotSubstitution,
    OrSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_ns = LaunchConfiguration("agent_ns")
    lead_agent = LaunchConfiguration("lead_agent")
    loc_comparison = LaunchConfiguration("loc_comparison")

    fleet_param_file = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "coug_bringup_params.yaml",
        ]
    )

    coug_des_dir = get_package_share_directory("coug_description")
    coug_des_launch_dir = os.path.join(coug_des_dir, "launch")
    coug_comms_dir = get_package_share_directory("coug_comms")
    coug_comms_launch_dir = os.path.join(coug_comms_dir, "launch")
    coug_fg_dir = get_package_share_directory("coug_fg")
    coug_fg_launch_dir = os.path.join(coug_fg_dir, "launch")
    coug_helm_dir = get_package_share_directory("coug_helm")
    coug_helm_launch_dir = os.path.join(coug_helm_dir, "launch")
    coug_belief_mppi_dir = get_package_share_directory("coug_belief_mppi")
    coug_belief_mppi_launch_dir = os.path.join(coug_belief_mppi_dir, "launch")
    coug_viz_dvl_dir = get_package_share_directory("coug_visual_dvl")
    coug_viz_dvl_launch_dir = os.path.join(coug_viz_dvl_dir, "launch")

    coug_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_des_launch_dir, "coug_description.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
        }.items(),
    )

    coug_comms_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_comms_launch_dir, "coug_comms_agent.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
        }.items(),
    )

    coug_fg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_fg_launch_dir, "coug_fg.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
            "lead_agent": lead_agent,
            "loc_comparison": loc_comparison,
        }.items(),
        condition=IfCondition(NotEqualsSubstitution(agent_ns, "coug2")),
    )

    coug_fg_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_fg_launch_dir, "coug_fg_ekf.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
        }.items(),
        condition=IfCondition(EqualsSubstitution(agent_ns, "coug2")),
    )

    coug_helm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_helm_launch_dir, "coug_helm.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
        }.items(),
        condition=IfCondition(
            NotSubstitution(
                OrSubstitution(
                    EqualsSubstitution(agent_ns, "blue1sim"),
                    OrSubstitution(
                        EqualsSubstitution(agent_ns, "blue2sim"),
                        EqualsSubstitution(agent_ns, "wamv1sim"),
                    ),
                )
            )
        ),
    )

    coug_belief_mppi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_belief_mppi_launch_dir, "coug_belief_mppi.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
        }.items(),
        condition=IfCondition(
            OrSubstitution(
                EqualsSubstitution(agent_ns, "blue1sim"),
                OrSubstitution(
                    EqualsSubstitution(agent_ns, "blue2sim"),
                    EqualsSubstitution(agent_ns, "wamv1sim"),
                ),
            )
        ),
    )

    coug_visual_dvl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_viz_dvl_launch_dir, "coug_visual_dvl.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
        }.items(),
        condition=IfCondition(
            OrSubstitution(
                EqualsSubstitution(agent_ns, "blue1sim"),
                EqualsSubstitution(agent_ns, "blue2sim"),
            )
        ),
    )

    bag_recorder_node = Node(
        package="coug_bringup",
        executable="bag_recorder",
        name="bag_recorder_node",
        parameters=[
            fleet_param_file,
            {
                "use_sim_time": use_sim_time,
                "agent_ns": agent_ns,
                "log_dir": launch_config.log_dir,
            },
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOG_DIR", launch_config.log_dir),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "agent_ns",
                default_value="auv0",
                description="Namespace for the agent (e.g. auv0)",
            ),
            DeclareLaunchArgument(
                "lead_agent",
                default_value="",
                description="Namespace of the lead agent (optional)",
            ),
            DeclareLaunchArgument(
                "loc_comparison",
                default_value="false",
                description="Launch additional localization nodes if true",
            ),
            GroupAction(
                actions=[
                    PushRosNamespace(agent_ns),
                    bag_recorder_node,
                    coug_comms_agent_launch,
                    coug_description_launch,
                    coug_fg_launch,
                    coug_fg_ekf_launch,
                    coug_helm_launch,
                    coug_belief_mppi_launch,
                    coug_visual_dvl_launch,
                ]
            ),
        ]
    )

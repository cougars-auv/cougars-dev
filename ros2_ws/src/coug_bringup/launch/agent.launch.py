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
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import launch_config
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace


def is_agent(agent_ns: LaunchConfiguration, *names: str) -> PythonExpression:
    return PythonExpression(["'", agent_ns, "' in ", str(names)])


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_ns = LaunchConfiguration("agent_ns")
    loc_comparison = LaunchConfiguration("loc_comparison")
    lead_agent = LaunchConfiguration("lead_agent")
    initial_position = LaunchConfiguration("initial_position")
    initial_orientation = LaunchConfiguration("initial_orientation")
    scenario_param_file = LaunchConfiguration("scenario_param_file")

    fleet_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_bringup_params.yaml"]
    )
    agent_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), [agent_ns, "_params.yaml"]]
    )
    resolved_scenario_param_file = PythonExpression(
        ["'", scenario_param_file, "' or '", agent_param_file, "'"]
    )

    coug_belief_mppi_dir = get_package_share_directory("coug_belief_mppi")
    coug_belief_mppi_launch_dir = os.path.join(coug_belief_mppi_dir, "launch")
    coug_comms_dir = get_package_share_directory("coug_comms")
    coug_comms_launch_dir = os.path.join(coug_comms_dir, "launch")
    coug_control_dir = get_package_share_directory("coug_control")
    coug_control_launch_dir = os.path.join(coug_control_dir, "launch")
    coug_description_dir = get_package_share_directory("coug_description")
    coug_description_launch_dir = os.path.join(coug_description_dir, "launch")
    coug_fg_dir = get_package_share_directory("coug_fg")
    coug_fg_launch_dir = os.path.join(coug_fg_dir, "launch")
    coug_helm_dir = get_package_share_directory("coug_helm")
    coug_helm_launch_dir = os.path.join(coug_helm_dir, "launch")
    coug_terrain_dir = get_package_share_directory("coug_terrain")
    coug_terrain_launch_dir = os.path.join(coug_terrain_dir, "launch")
    coug_visual_dvl_dir = get_package_share_directory("coug_visual_dvl")
    coug_visual_dvl_launch_dir = os.path.join(coug_visual_dvl_dir, "launch")

    coug_belief_mppi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_belief_mppi_launch_dir, "coug_belief_mppi.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
            "scenario_param_file": scenario_param_file,
        }.items(),
        condition=IfCondition(
            is_agent(agent_ns, "blue1holo", "wamv1holo", "rover1gz", "rover2gz", "rover3gz")
        ),
    )

    coug_comms_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_comms_launch_dir, "coug_comms_agent.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
            "scenario_param_file": scenario_param_file,
        }.items(),
    )

    coug_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_control_launch_dir, "coug_control.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
            "scenario_param_file": scenario_param_file,
        }.items(),
    )

    coug_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_description_launch_dir, "coug_description.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
            "scenario_param_file": scenario_param_file,
        }.items(),
    )

    coug_fg_dual_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_fg_launch_dir, "coug_fg_dual_ekf.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
            "scenario_param_file": scenario_param_file,
            "initial_position": initial_position,
            "initial_orientation": initial_orientation,
        }.items(),
        condition=IfCondition(is_agent(agent_ns, "rover1gz", "rover2gz", "rover3gz")),
    )

    coug_fg_dvl_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_fg_launch_dir, "coug_fg_dvl_ekf.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
            "scenario_param_file": scenario_param_file,
            "initial_position": initial_position,
            "initial_orientation": initial_orientation,
        }.items(),
        condition=IfCondition(is_agent(agent_ns, "coug2")),
    )

    coug_fg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(coug_fg_launch_dir, "coug_fg.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
            "scenario_param_file": scenario_param_file,
            "loc_comparison": loc_comparison,
            "lead_agent": lead_agent,
            "initial_position": initial_position,
            "initial_orientation": initial_orientation,
        }.items(),
        condition=UnlessCondition(is_agent(agent_ns, "coug2", "rover1gz", "rover2gz", "rover3gz")),
    )

    coug_helm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(coug_helm_launch_dir, "coug_helm.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
            "scenario_param_file": scenario_param_file,
        }.items(),
    )

    coug_terrain_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_terrain_launch_dir, "coug_terrain.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
            "scenario_param_file": scenario_param_file,
        }.items(),
        condition=IfCondition(is_agent(agent_ns, "rover1gz", "rover2gz", "rover3gz", "wamv1holo")),
    )

    coug_visual_dvl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_visual_dvl_launch_dir, "coug_visual_dvl.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "agent_ns": agent_ns,
            "scenario_param_file": scenario_param_file,
        }.items(),
        condition=IfCondition(is_agent(agent_ns, "blue1holo")),
    )

    bag_recorder_node = Node(
        package="coug_bringup",
        executable="bag_recorder",
        name="bag_recorder_node",
        parameters=[
            fleet_param_file,
            agent_param_file,
            resolved_scenario_param_file,
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
            ),
            DeclareLaunchArgument(
                "agent_ns",
                default_value="auv0",
            ),
            DeclareLaunchArgument(
                "scenario_param_file",
                default_value="",
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
                "initial_position",
                default_value="",
            ),
            DeclareLaunchArgument(
                "initial_orientation",
                default_value="",
            ),
            GroupAction(
                actions=[
                    PushRosNamespace(agent_ns),
                    bag_recorder_node,
                    coug_belief_mppi_launch,
                    coug_comms_agent_launch,
                    coug_control_launch,
                    coug_description_launch,
                    coug_fg_dual_ekf_launch,
                    coug_fg_dvl_ekf_launch,
                    coug_fg_launch,
                    coug_helm_launch,
                    coug_terrain_launch,
                    coug_visual_dvl_launch,
                ]
            ),
        ]
    )

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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    EqualsSubstitution,
    NotEqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    use_sim_time = LaunchConfiguration("use_sim_time")
    auv_urdf = LaunchConfiguration("auv_urdf")
    auv_ns = LaunchConfiguration("auv_ns")
    compare = LaunchConfiguration("compare")
    set_origin = LaunchConfiguration("set_origin")

    urdf_file = PathJoinSubstitution(
        [
            FindPackageShare("coug_description"),
            "urdf",
            auv_urdf,
        ]
    )

    coug_des_dir = get_package_share_directory("coug_description")
    coug_des_launch_dir = os.path.join(coug_des_dir, "launch")
    coug_fgo_dir = get_package_share_directory("coug_fgo")
    coug_fgo_launch_dir = os.path.join(coug_fgo_dir, "launch")
    coug_helm_dir = get_package_share_directory("coug_helm")
    coug_helm_launch_dir = os.path.join(coug_helm_dir, "launch")
    coug_active_fgo_dir = get_package_share_directory("coug_active_fgo")
    coug_active_fgo_launch_dir = os.path.join(coug_active_fgo_dir, "launch")
    coug_viz_dvl_dir = get_package_share_directory("coug_visual_dvl")
    coug_viz_dvl_launch_dir = os.path.join(coug_viz_dvl_dir, "launch")

    coug_des_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_des_launch_dir, "coug_description.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "urdf_file": urdf_file,
            "auv_ns": auv_ns,
        }.items(),
    )

    coug_fgo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_fgo_launch_dir, "coug_fgo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "auv_ns": auv_ns,
            "compare": compare,
            "set_origin": set_origin,
        }.items(),
        condition=IfCondition(
            AndSubstitution(
                NotEqualsSubstitution(auv_ns, "coug2_dvldr"),
                NotEqualsSubstitution(auv_ns, "coug2_ekf"),
            )
        ),
    )

    coug_fgo_dvldr_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_fgo_launch_dir, "coug_fgo_dvldr.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "auv_ns": auv_ns,
            "set_origin": set_origin,
        }.items(),
        condition=IfCondition(EqualsSubstitution(auv_ns, "coug2_dvldr")),
    )

    coug_fgo_ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_fgo_launch_dir, "coug_fgo_ekf.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "auv_ns": auv_ns,
            "set_origin": set_origin,
        }.items(),
        condition=IfCondition(EqualsSubstitution(auv_ns, "coug2_ekf")),
    )

    coug_helm_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_helm_launch_dir, "coug_helm.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "auv_ns": auv_ns,
        }.items(),
        condition=IfCondition(
            NotEqualsSubstitution(LaunchConfiguration("auv_ns"), "blue0sim")
        ),
    )

    coug_active_fgo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_active_fgo_launch_dir, "coug_active_fgo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "auv_ns": auv_ns,
        }.items(),
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("auv_ns"), "blue0sim")
        ),
    )

    coug_viz_dvl_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_viz_dvl_launch_dir, "coug_visual_dvl.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "auv_ns": auv_ns,
        }.items(),
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("auv_ns"), "blue0sim")
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "auv_urdf",
                default_value="couguv_holoocean.urdf.xacro",
                description="URDF filename in coug_description/urdf",
            ),
            DeclareLaunchArgument(
                "auv_ns",
                default_value="auv0",
                description="Namespace for the AUV (e.g. auv0)",
            ),
            DeclareLaunchArgument(
                "compare",
                default_value="false",
                description="Launch additional localization nodes if true",
            ),
            DeclareLaunchArgument(
                "set_origin",
                default_value="true",
                description="Whether navsat_odom_node owns and publishes the ENU origin",
            ),
            GroupAction(
                actions=[
                    PushRosNamespace(auv_ns),
                    coug_des_cmd,
                    coug_fgo_cmd,
                    coug_fgo_dvldr_cmd,
                    coug_fgo_ekf_cmd,
                    coug_helm_cmd,
                    coug_active_fgo_cmd,
                    coug_viz_dvl_cmd,
                ]
            ),
        ]
    )

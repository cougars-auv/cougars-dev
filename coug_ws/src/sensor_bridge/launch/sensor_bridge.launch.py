# Copyright (c) 2026 BYU FRoSt Lab
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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    auv_ns = LaunchConfiguration("auv_ns", default="auv0")

    pkg_share = get_package_share_directory("sensor_bridge")
    params_file = os.path.join(pkg_share, "config", "sensor_bridge_params.yaml")

    dvl_link_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/dvl_link' if '",
            auv_ns,
            "' != '' else 'dvl_link'",
        ]
    )

    base_link_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/base_link' if '",
            auv_ns,
            "' != '' else 'base_link'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (HoloOcean) clock if true",
            ),
            DeclareLaunchArgument(
                "auv_ns",
                default_value="auv0",
                description=(
                    "Namespace for the AUV (e.g. auv0), used for namespacing topics and frames"
                ),
            ),
            Node(
                package="sensor_bridge",
                executable="dvl_twist_converter",
                name="dvl_twist_converter_node",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "frame_id": dvl_link_frame,
                    },
                ],
            ),
            Node(
                package="sensor_bridge",
                executable="dvl_odom_converter",
                name="dvl_odom_converter_node",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "child_frame_id": dvl_link_frame,
                    },
                ],
            ),
            Node(
                package="sensor_bridge",
                executable="gps_odom_converter",
                name="gps_odom_converter_node",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "base_frame_id": base_link_frame,
                    },
                ],
            ),
        ]
    )

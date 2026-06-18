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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)


def generate_launch_description() -> LaunchDescription:

    auv_ns = LaunchConfiguration("auv_ns")

    seatrac_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "seatrac_params.yaml",
        ]
    )
    sbg_driver_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "sbg_driver_params.yaml",
        ]
    )
    dvl_a50_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "dvl_a50_params.yaml",
        ]
    )
    pressure_sensor_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "pressure_sensor_params.yaml",
        ]
    )
    gps_tools_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "gps_tools_params.yaml",
        ]
    )
    gpsd_client_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "gpsd_client_params.yaml",
        ]
    )
    nmea_gpsd_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "nmea_gpsd_params.yaml",
        ]
    )
    ntrip_client_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "ntrip_client_params.yaml",
        ]
    )
    auv_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            PythonExpression(["'", auv_ns, "' + '_params.yaml'"]),
        ]
    )

    # TODO: Add direct imports to package.xml and .repos files

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "auv_ns",
                default_value="auv0",
                description="Namespace for the AUV (e.g. auv0)",
            ),
            Node(
                package="seatrac",
                executable="modem",
                name="modem",
                parameters=[seatrac_params, auv_params],
            ),
            Node(
                package="sbg_driver",
                executable="sbg_device",
                name="sbg_device",
                parameters=[sbg_driver_params, auv_params],
            ),
            Node(
                package="dvl_a50",
                executable="dvl_a50_sensor",
                name="dvl_a50_sensor",
                parameters=[dvl_a50_params, auv_params],
            ),
            Node(
                package="pressure_sensor",
                executable="pressure_pub",
                name="pressure_pub",
                parameters=[pressure_sensor_params, auv_params],
            ),
            Node(
                package="gps_tools",
                executable="utm_odometry_node",
                name="utm_odometry_node",
                parameters=[gps_tools_params, auv_params],
            ),
            Node(
                package="gpsd_client",
                executable="gpsd_client",
                name="gpsd_client",
                parameters=[gpsd_client_params, auv_params],
            ),
            Node(
                package="nmea_gpsd",
                executable="nmea_gpsd_udp",
                name="nmea_gpsd_udp",
                parameters=[nmea_gpsd_params, auv_params],
            ),
            Node(
                package="ntrip_client",
                executable="ntrip_ros.py",
                name="ntrip_client",
                parameters=[ntrip_client_params, auv_params],
            ),
        ]
    )

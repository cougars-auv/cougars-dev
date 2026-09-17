#!/bin/bash
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

set -e

source "${OVERLAY_WS}/install/setup.bash"

ros2 topic pub --once /origin sensor_msgs/msg/NavSatFix "{
  header: {frame_id: 'map'},
  status: {status: 0},
  latitude: 40.33940,
  longitude: -111.90721,
  altitude: 1412.0
}"

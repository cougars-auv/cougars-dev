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

# --- Selection ---
scenario=$(gum choose --header "Choose a HoloOcean scenario:" \
  "CougUV" \
  "BlueROV2" \
  "WAM-V" \
  "CougUV Multi-Agent" \
  "BlueROV2 Multi-Agent" \
  "Mixed Multi-Agent") || exit 0

case ${scenario} in
  "CougUV") params="couguv_openwater_params.yaml";;
  "BlueROV2") params="bluerov2_openwater_params.yaml";;
  "WAM-V") params="wamv_openwater_params.yaml";;
  "CougUV Multi-Agent") params="multi_couguv_openwater_params.yaml";;
  "BlueROV2 Multi-Agent") params="multi_bluerov2_openwater_params.yaml";;
  "Mixed Multi-Agent") params="multi_mixed_openwater_params.yaml";;
esac

# --- Launch ---
docker exec -it --user ue4 cougars-holoocean-ct /bin/bash -c \
  "source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/ue4/ros2_ws/install/setup.bash \
  && ros2 run holoocean_main holoocean_node --ros-args --params-file /home/ue4/config/holoocean/${params}"

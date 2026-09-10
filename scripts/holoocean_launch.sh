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
scenario=$(basename -a "${CONFIG_DIR}"/holoocean/*.json | sed 's/.json$//' | sort |
  gum choose --header "Choose a HoloOcean scenario:") || exit 0

# --- Launch ---
params_file="/home/ue4/config/holoocean/${scenario}_params.yaml"

docker exec -it --user ue4 cougars-holoocean-ct /bin/bash -c \
  "source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/ue4/ros2_ws/install/setup.bash \
  && ros2 run holoocean_main holoocean_node --ros-args --params-file ${params_file}"

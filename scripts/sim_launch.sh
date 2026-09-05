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

# --- Selection ---
scenario=$(gum choose --header "Choose a HoloOcean scenario:" \
  "CougUV" \
  "BlueROV2" \
  "WAM-V" \
  "CougUV Multi-Agent" \
  "BlueROV2 Multi-Agent" \
  "Mixed Multi-Agent") || exit 0

case ${scenario} in
  "CougUV") selected_agents=(coug1sim);;
  "BlueROV2") selected_agents=(blue1sim);;
  "WAM-V") selected_agents=(wamv1sim);;
  "CougUV Multi-Agent") selected_agents=(coug1sim coug2sim coug3sim);;
  "BlueROV2 Multi-Agent") selected_agents=(blue1sim blue2sim);;
  "Mixed Multi-Agent") selected_agents=(wamv1sim blue1sim coug2sim);;
esac

agent_list="[$(printf '%s\n' "${selected_agents[@]}" | paste -sd, | sed 's/,/, /g')]"

# --- Options ---
options=$(gum choose --no-limit --header "Select options:" \
  "Record rosbag" \
  "Localization comparison" \
  "Disable sensor noise" \
  "Enable voxblox mapping" \
  "Enable shared voxblox mapping" \
  "Specify lead agent" \
  "Acomms simulation" \
  "HITL mode") || exit 0

record_bag_path=""
loc_comparison="false"
add_noise="true"
enable_mapping="false"
enable_shared_mapping="false"
lead_agent=""
enable_direct_comms="true"
enable_acoustic_comms="true"
hitl_mode="false"

if [[ "${options}" == *"Record rosbag"* ]]; then
  suffix=$(gum input --placeholder "Set bag suffix..." || true)
  record_bag_path="${BAGS_DIR}/${suffix:-rosbag}$(date +'_%Y-%m-%d-%H-%M-%S')"
fi

if [[ "${options}" == *"Localization comparison"* ]]; then
  loc_comparison="true"
fi

if [[ "${options}" == *"Disable sensor noise"* ]]; then
  add_noise="false"
fi

if [[ "${options}" == *"Enable voxblox mapping"* ]]; then
  enable_mapping="true"
fi

if [[ "${options}" == *"Enable shared voxblox mapping"* ]]; then
  enable_shared_mapping="true"
fi

if [[ "${options}" == *"Specify lead agent"* ]]; then
  lead_agent=$(gum choose --header "Select lead agent:" "${selected_agents[@]}") || exit 0
fi

if [[ "${options}" == *"Acomms simulation"* ]]; then
  enable_direct_comms="false"
fi

if [[ "${options}" == *"HITL mode"* ]]; then
  hitl_mode="true"
fi

# --- Launch ---
launch_args=(
  "agent_list:=${agent_list}"
)
if [[ -n ${lead_agent} ]]; then
  launch_args+=("lead_agent:=${lead_agent}")
fi
if [[ -n ${record_bag_path} ]]; then
  launch_args+=("record_bag_path:=${record_bag_path}")
fi
launch_args+=(
  "loc_comparison:=${loc_comparison}"
  "add_noise:=${add_noise}"
  "enable_mapping:=${enable_mapping}"
  "enable_shared_mapping:=${enable_shared_mapping}"
)
launch_args+=(
  "enable_direct_comms:=${enable_direct_comms}"
  "enable_acoustic_comms:=${enable_acoustic_comms}"
  "hitl_mode:=${hitl_mode}"
)

echo "ros2 launch coug_bringup sim.launch.py ${launch_args[*]}"
ros2 launch coug_bringup sim.launch.py "${launch_args[@]}"

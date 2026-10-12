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
bag_name=$(cd "${BAGS_DIR}" && find . -name metadata.yaml -printf '%h\n' |
  sed 's|^\./||' | sort -r |
  gum filter --placeholder "Select a bag to play...") || exit 0
[[ -z ${bag_name} ]] && exit 0
play_bag_path="${BAGS_DIR}/${bag_name}"

while true; do
  selected_agents=$(basename -s _params.yaml -a "${CONFIG_DIR}"/*_params.yaml |
    gum choose --no-limit --header "Select agents to launch:") || exit 0
  [[ -n ${selected_agents} ]] && break
done

mapfile -t selected_agents <<<"${selected_agents}"
agent_list=$(printf '%s,' "${selected_agents[@]}")
agent_list="[${agent_list%,}]"

# --- Options ---
options=$(gum choose --no-limit --header "Select options:" \
  "Record rosbag" \
  "Set start offset" \
  "Set playback duration" \
  "Set playback rate" \
  "Start paused" \
  "Localization comparison" \
  "Specify lead agent" \
  "Enable voxblox mapping" \
  "HITL mode") || exit 0

launch_args=(
  "agent_list:=${agent_list}"
  "play_bag_path:=${play_bag_path}"
)

if [[ "${options}" == *"Record rosbag"* ]]; then
  prefix=$(gum input --placeholder "Set bag prefix..." || true)
  prefix=${prefix//[^A-Za-z0-9_-]/_}
  launch_args+=("record_bag_path:=${BAGS_DIR}/${prefix:-rosbag}$(date +'_%Y-%m-%d-%H-%M-%S')")
fi

if [[ "${options}" == *"Set start offset"* ]]; then
  start_offset=$(gum input --placeholder "Set start offset (s)..." || true)
  if [[ "${start_offset}" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    launch_args+=("start_offset:=${start_offset}")
  fi
fi

if [[ "${options}" == *"Set playback duration"* ]]; then
  playback_duration=$(gum input --placeholder "Set playback duration (s; -1 for full bag)..." || true)
  if [[ "${playback_duration}" =~ ^[0-9]+(\.[0-9]+)?$|^-1(\.0+)?$ ]]; then
    launch_args+=("playback_duration:=${playback_duration}")
  fi
fi

if [[ "${options}" == *"Set playback rate"* ]]; then
  playback_rate=$(gum input --placeholder "Set playback rate (e.g. 0.5, 2.0)..." || true)
  if [[ "${playback_rate}" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    launch_args+=("playback_rate:=${playback_rate}")
  fi
fi

if [[ "${options}" == *"Start paused"* ]]; then
  launch_args+=("start_paused:=true")
fi

if [[ "${options}" == *"Localization comparison"* ]]; then
  launch_args+=("loc_comparison:=true")
fi

if [[ "${options}" == *"Specify lead agent"* ]]; then
  lead_agent=$(gum choose --header "Select lead agent:" "${selected_agents[@]}") || exit 0
  launch_args+=("lead_agent:=${lead_agent}")
fi

if [[ "${options}" == *"Enable voxblox mapping"* ]]; then
  launch_args+=("enable_mapping:=true")
fi

if [[ "${options}" == *"HITL mode"* ]]; then
  launch_args+=("hitl_mode:=true")
fi

# --- Launch ---
echo "ros2 launch coug_bringup bag.launch.py ${launch_args[*]}"
ros2 launch coug_bringup bag.launch.py "${launch_args[@]}"

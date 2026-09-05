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
bag_name=$(cd "${BAGS_DIR}" && find . -name "metadata.yaml" -exec dirname {} \; |
  sed 's|^\./||' | sort -r |
  gum filter --placeholder "Select a bag to play...") || exit 0
[[ -z ${bag_name} ]] && exit 0
play_bag_path="${BAGS_DIR}/${bag_name}"

agent_ns=$(basename -a "${CONFIG_DIR}"/*_params.yaml |
  sed 's/_params.yaml$//' | sort |
  gum filter --placeholder "Select an agent to launch...") || exit 0
[[ -z ${agent_ns} ]] && exit 0
agent_list="[${agent_ns}]"

# --- Options ---
options=$(gum choose --no-limit --header "Select options:" \
  "Record rosbag" \
  "Set start delay" \
  "Set playback rate" \
  "Localization comparison" \
  "HITL mode") || exit 0

record_bag_path=""
start_delay="0.0"
playback_rate="1.0"
loc_comparison="false"
hitl_mode="false"

if [[ "${options}" == *"Record rosbag"* ]]; then
  suffix=$(gum input --placeholder "Set bag suffix..." || true)
  record_bag_path="${BAGS_DIR}/${suffix:-rosbag}$(date +'_%Y-%m-%d-%H-%M-%S')"
fi

if [[ "${options}" == *"Set start delay"* ]]; then
  start_delay=$(gum input --placeholder "Set start delay (s)..." || echo "0.0")
  if ! [[ "${start_delay}" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    start_delay="0.0"
  fi
fi

if [[ "${options}" == *"Set playback rate"* ]]; then
  playback_rate=$(gum input --placeholder "Set playback rate (e.g. 0.5, 2.0)..." || echo "1.0")
  if ! [[ "${playback_rate}" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    playback_rate="1.0"
  fi
fi

if [[ "${options}" == *"Localization comparison"* ]]; then
  loc_comparison="true"
fi

if [[ "${options}" == *"HITL mode"* ]]; then
  hitl_mode="true"
fi

# --- Launch ---
launch_args=(
  "agent_list:=${agent_list}"
  "play_bag_path:=${play_bag_path}"
)
if [[ -n ${record_bag_path} ]]; then
  launch_args+=("record_bag_path:=${record_bag_path}")
fi
launch_args+=(
  "start_delay:=${start_delay}"
  "playback_rate:=${playback_rate}"
  "loc_comparison:=${loc_comparison}"
  "hitl_mode:=${hitl_mode}"
)

echo "ros2 launch coug_bringup bag.launch.py ${launch_args[*]}"
ros2 launch coug_bringup bag.launch.py "${launch_args[@]}"

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
  gum filter --placeholder "Select a bag to visualize...") || exit 0
[[ -z ${bag_name} ]] && exit 0
play_bag_path="${BAGS_DIR}/${bag_name}"

agent_ns=$(basename -a "${CONFIG_DIR}"/*_params.yaml |
  sed 's/_params.yaml$//' | sort |
  gum filter --placeholder "Select an agent to visualize...") || exit 0
[[ -z ${agent_ns} ]] && exit 0
agent_list="[${agent_ns}]"

# --- Options ---
options=$(gum choose --no-limit --header "Select options:" \
  "Set start offset" \
  "Set playback duration" \
  "Set playback rate" \
  "Start paused") || exit 0

start_offset="0.0"
playback_duration="-1.0"
playback_rate="1.0"
start_paused="false"

if [[ "${options}" == *"Set start offset"* ]]; then
  start_offset=$(gum input --placeholder "Set start offset (s)..." || echo "0.0")
  if ! [[ "${start_offset}" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    start_offset="0.0"
  fi
fi

if [[ "${options}" == *"Set playback duration"* ]]; then
  playback_duration=$(gum input --placeholder "Set playback duration (s; -1 for full bag)..." || echo "-1.0")
  if ! [[ "${playback_duration}" =~ ^[0-9]+(\.[0-9]+)?$|^-1(\.0+)?$ ]]; then
    playback_duration="-1.0"
  fi
fi

if [[ "${options}" == *"Set playback rate"* ]]; then
  playback_rate=$(gum input --placeholder "Set playback rate (e.g. 0.5, 2.0)..." || echo "1.0")
  if ! [[ "${playback_rate}" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    playback_rate="1.0"
  fi
fi

if [[ "${options}" == *"Start paused"* ]]; then
  start_paused="true"
fi

# --- Launch ---
launch_args=(
  "agent_list:=${agent_list}"
  "play_bag_path:=${play_bag_path}"
  "start_offset:=${start_offset}"
  "playback_duration:=${playback_duration}"
  "playback_rate:=${playback_rate}"
  "start_paused:=${start_paused}"
)

echo "ros2 launch coug_bringup viz.launch.py ${launch_args[*]}"
ros2 launch coug_bringup viz.launch.py "${launch_args[@]}"

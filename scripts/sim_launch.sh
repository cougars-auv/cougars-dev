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
scenario_param_file=$(cd "${CONFIG_DIR}" &&
  printf '%s\n' holoocean/*_params.yaml gazebo/*_params.yaml |
  gum choose --header "Select a simulation scenario:") || exit 0
scenario_param_file="${CONFIG_DIR}/${scenario_param_file}"

if [[ ${scenario_param_file} == */gazebo/* ]]; then
  option_list=(
    "Record rosbag"
    "Enable voxblox mapping"
    "HITL mode"
  )
else
  option_list=(
    "Record rosbag"
    "Disable sensor noise"
    "Localization comparison"
    "Specify lead agent"
    "Acomms simulation"
    "Unknown initial poses"
    "Enable voxblox mapping"
    "HITL mode"
  )
fi

# --- Options ---
options=$(gum choose --no-limit --header "Select options:" \
  "${option_list[@]}") || exit 0

launch_args=("scenario_param_file:=${scenario_param_file}")

if [[ "${options}" == *"Record rosbag"* ]]; then
  prefix=$(gum input --placeholder "Set bag prefix..." || true)
  launch_args+=("record_bag_path:=${BAGS_DIR}/${prefix:-rosbag}$(date +'_%Y-%m-%d-%H-%M-%S')")
fi

if [[ "${options}" == *"Disable sensor noise"* ]]; then
  launch_args+=("add_noise:=false")
fi

if [[ "${options}" == *"Localization comparison"* ]]; then
  launch_args+=("loc_comparison:=true")
fi

if [[ "${options}" == *"Specify lead agent"* ]]; then
  scenario_file=$(sed -n 's/.*scenario_file: "\(.*\)".*/\1/p' "${scenario_param_file}")
  lead_agent=$(jq -r '.agents[].agent_name | select(. != "base_station")' \
    "${CONFIG_DIR}/holoocean/${scenario_file}" |
    gum choose --header "Select lead agent:") || exit 0
  launch_args+=("lead_agent:=${lead_agent}")
fi

if [[ "${options}" == *"Acomms simulation"* ]]; then
  launch_args+=("enable_direct_comms:=false")
fi

if [[ "${options}" == *"Unknown initial poses"* ]]; then
  launch_args+=("use_spawn_pose:=false")
fi

if [[ "${options}" == *"Enable voxblox mapping"* ]]; then
  launch_args+=("enable_mapping:=true")
fi

if [[ "${options}" == *"HITL mode"* ]]; then
  launch_args+=("hitl_mode:=true")
fi

# --- Launch ---
echo "ros2 launch coug_bringup sim.launch.py ${launch_args[*]}"
ros2 launch coug_bringup sim.launch.py "${launch_args[@]}"

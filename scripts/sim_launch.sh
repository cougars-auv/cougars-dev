#!/bin/bash
set -e

source ${OVERLAY_WS}/install/setup.bash

# --- Selection ---
scenario=$(gum choose --header "Choose a HoloOcean scenario:" "CougUV" "BlueROV2" "WAM-V" "Multi-Agent")

case ${scenario} in
  "CougUV") agent_list="[coug1sim]";;
  "BlueROV2") agent_list="[blue1sim]";;
  "WAM-V") agent_list="[wamv1sim]";;
  "Multi-Agent") agent_list="[coug1sim, coug2sim, coug3sim]";;
esac

# --- Options ---
options=$(gum choose --no-limit --header "Select options:" "Record rosbag" "Localization comparison" "Disable sensor noise" "Specify lead agent" "Acomms simulation" "HITL mode") || exit 0

record_bag_path=""
loc_comparison="false"
add_noise="true"
lead_agent=""
enable_direct_comms="true"
enable_acoustic_comms="true"
hitl_mode="false"

if [[ "${options}" == *"Record rosbag"* ]]; then
  suffix=$(gum input --placeholder "Set bag suffix..." || echo "")
  if [ -n "${suffix}" ]; then
    record_bag_path="${BAGS_DIR}/${suffix}$(date +'_%Y-%m-%d-%H-%M-%S')"
  else
    record_bag_path="${BAGS_DIR}/rosbag$(date +'_%Y-%m-%d-%H-%M-%S')"
  fi
fi

if [[ "${options}" == *"Localization comparison"* ]]; then
  loc_comparison="true"
fi

if [[ "${options}" == *"Disable sensor noise"* ]]; then
  add_noise="false"
fi

if [[ "${options}" == *"Acomms simulation"* ]]; then
  enable_direct_comms="false"
fi

if [[ "${options}" == *"HITL mode"* ]]; then
  hitl_mode="true"
fi

if [[ "${options}" == *"Specify lead agent"* ]]; then
  agents_raw=$(echo "${agent_list}" | tr -d '[]' | tr -d ',')
  lead_agent=$(gum choose --header "Select lead agent:" ${agents_raw}) || exit 0
fi

# --- Launch ---
args=(
  "agent_list:=${agent_list}"
  "loc_comparison:=${loc_comparison}"
  "add_noise:=${add_noise}"
  "enable_direct_comms:=${enable_direct_comms}"
  "enable_acoustic_comms:=${enable_acoustic_comms}"
  "hitl_mode:=${hitl_mode}"
)
if [ -n "${lead_agent}" ]; then
  args+=("lead_agent:=${lead_agent}")
fi
if [ -n "${record_bag_path}" ]; then
  args+=("record_bag_path:=${record_bag_path}")
fi

echo "ros2 launch coug_bringup sim.launch.py ${args[*]}"
ros2 launch coug_bringup sim.launch.py "${args[@]}"

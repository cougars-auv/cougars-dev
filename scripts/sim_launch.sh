#!/bin/bash
set -e

source ${OVERLAY_WS}/install/setup.bash

# --- Selection ---
scenario=$(gum choose --header "Choose a HoloOcean scenario:" "CougUV" "BlueROV2" "Multi-Agent")

base_station="true"
case ${scenario} in
  "CougUV") agent_list="[coug1sim]";;
  "BlueROV2") agent_list="[blue1sim]"; base_station="false";;
  "Multi-Agent") agent_list="[coug1sim, coug2sim, coug3sim]";;
esac

# --- Options ---
options=$(gum choose --no-limit --header "Select options:" "Record rosbag" "Localization comparison" "Disable sensor noise" "Acomms simulation" "HITL mode") || exit 0

loc_comparison="false"
record_bag_path=""
add_noise="true"
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

# --- Launch ---
args=(
  "agent_list:=${agent_list}"
  "loc_comparison:=${loc_comparison}"
  "add_noise:=${add_noise}"
  "base_station:=${base_station}"
  "enable_direct_comms:=${enable_direct_comms}"
  "enable_acoustic_comms:=${enable_acoustic_comms}"
  "hitl_mode:=${hitl_mode}"
)
[ -n "${record_bag_path}" ] && args+=("record_bag_path:=${record_bag_path}")

echo "ros2 launch coug_bringup sim.launch.py ${args[*]}"
if [ -n "${record_bag_path}" ]; then
  tmp=$(mktemp -d)
  ROS_LOG_DIR="${tmp}" ros2 launch coug_bringup sim.launch.py "${args[@]}"

  if [ -d "${record_bag_path}" ]; then
    mv "${tmp}" "${record_bag_path}/log"
  else
    rm -rf "${tmp}"
  fi
else
  ros2 launch coug_bringup sim.launch.py "${args[@]}"
fi

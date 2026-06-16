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
options=$(gum choose --no-limit --header "Select options:" "Record rosbag" "Launch comparison methods" "Disable sensor noise" "Acoustic comms") || exit 0

compare="false"
record_bag_path=""
add_noise="true"
force_acomms="false"

if [[ "${options}" == *"Launch comparison methods"* ]]; then
  compare="true"
fi

if [[ "${options}" == *"Disable sensor noise"* ]]; then
  add_noise="false"
fi

if [[ "${options}" == *"Acoustic comms"* ]]; then
  force_acomms="true"
fi

if [[ "${options}" == *"Record rosbag"* ]]; then
  suffix=$(gum input --placeholder "Set bag suffix..." || echo "")
  if [ -n "${suffix}" ]; then
    record_bag_path="${BAGS_DIR}/${suffix}$(date +'_%Y-%m-%d-%H-%M-%S')"
  else
    record_bag_path="${BAGS_DIR}/rosbag$(date +'_%Y-%m-%d-%H-%M-%S')"
  fi
fi

# --- Launch ---
args=(
  "agent_list:=${agent_list}"
  "compare:=${compare}"
  "add_noise:=${add_noise}"
  "base_station:=${base_station}"
  "force_acomms:=${force_acomms}"
)
[ -n "${record_bag_path}" ] && args+=("record_bag_path:=${record_bag_path}")

echo "ros2 launch coug_bringup sim.launch.py ${args[*]}"
if [ -n "${record_bag_path}" ]; then
  tmp=$(mktemp -d)
  ROS_LOG_DIR="${tmp}" ros2 launch coug_bringup sim.launch.py "${args[@]}"

  if [ -d "${record_bag_path}" ]; then
    mv "${tmp}" "${record_bag_path}/log"

    mkdir -p "${record_bag_path}/config"
    if [ -d "${CONFIG_DIR}" ] && [ -n "$(ls -A "${CONFIG_DIR}")" ]; then
      cp -r "${CONFIG_DIR}/"* "${record_bag_path}/config/" 2>/dev/null || true
    fi
  else
    rm -rf "${tmp}"
  fi
else
  ros2 launch coug_bringup sim.launch.py "${args[@]}"
fi

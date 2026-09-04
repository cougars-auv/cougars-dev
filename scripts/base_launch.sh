#!/bin/bash
set -e

source "${OVERLAY_WS}/install/setup.bash"

# --- Selection ---
while true; do
  selected_agents=$(basename -a "${CONFIG_DIR}"/*_params.yaml | \
    sed 's/_params.yaml$//' | sort | \
    gum choose --no-limit --header "Select agents to launch:") || exit 0
  [[ -n ${selected_agents} ]] && break
done

mapfile -t selected_agents <<< "${selected_agents}"
agent_list="[$(printf '%s\n' "${selected_agents[@]}" | paste -sd, | sed 's/,/, /g')]"

# --- Options ---
options=$(gum choose --no-limit --header "Select options:" \
  "Record rosbag" \
  "Specify lead agent") || exit 0

record_bag_path=""
lead_agent=""

if [[ "${options}" == *"Record rosbag"* ]]; then
  suffix=$(gum input --placeholder "Set bag suffix..." || true)
  record_bag_path="${BAGS_DIR}/${suffix:-rosbag}$(date +'_%Y-%m-%d-%H-%M-%S')"
fi

if [[ "${options}" == *"Specify lead agent"* ]]; then
  lead_agent=$(gum choose --header "Select lead agent:" "${selected_agents[@]}") || exit 0
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

echo "ros2 launch coug_bringup base.launch.py ${launch_args[*]}"
ros2 launch coug_bringup base.launch.py "${launch_args[@]}"

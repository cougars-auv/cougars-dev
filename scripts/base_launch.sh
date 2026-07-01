#!/bin/bash
set -e

source ${OVERLAY_WS}/install/setup.bash

# --- Selection ---
while true; do
  selection=$(basename -a "${CONFIG_DIR}"/*_params.yaml | sed 's/_params.yaml$//' | sort | gum choose --no-limit --header "Select agents to launch:") || exit 0
  [ -n "${selection}" ] && break
done

agent_list="[$(paste -sd, <<< "${selection}" | sed 's/,/, /g')]"

# --- Options ---
options=$(gum choose --no-limit --header "Select options:" "Specify lead agent") || exit 0

lead_agent=""
if [[ "${options}" == *"Specify lead agent"* ]]; then
  lead_agent=$(gum choose --header "Select lead agent:" ${selection}) || exit 0
fi

# --- Launch ---
args=(
  "agent_list:=${agent_list}"
  "lead_agent:=${lead_agent}"
)

echo "ros2 launch coug_bringup base.launch.py ${args[*]}"
ros2 launch coug_bringup base.launch.py "${args[@]}"

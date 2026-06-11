#!/bin/bash
set -e

source ${OVERLAY_WS}/install/setup.bash

# --- Selection ---
while true; do
  selection=$(basename -a "${CONFIG_DIR}"/*_params.yaml | sed 's/_params.yaml$//' | sort | gum choose --no-limit --header "Select agents to launch:") || exit 0
  [ -n "${selection}" ] && break
done

agent_list="[$(paste -sd, <<< "${selection}" | sed 's/,/, /g')]"

# --- Launch ---
ros2 launch coug_bringup base.launch.py "agent_list:=${agent_list}"

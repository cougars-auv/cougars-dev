#!/bin/bash
set -e

source ${OVERLAY_WS}/install/setup.bash
source "$(dirname "$0")/utils/common.sh"

# --- Selection ---
while true; do
  selection=$(printf "%s\n" "${!AGENTS[@]}" | sort | gum choose --no-limit --header "Select agents to launch:") || exit 0
  [ -n "${selection}" ] && break
done
agent_list=$(agents_yaml ${selection})

# --- Launch ---
ros2 launch coug_bringup base.launch.py "agent_list:=${agent_list}"

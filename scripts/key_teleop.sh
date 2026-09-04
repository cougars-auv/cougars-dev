#!/bin/bash
set -e

source "${OVERLAY_WS}/install/setup.bash"

# --- Selection ---
agent_ns=$(basename -a "${CONFIG_DIR}"/*_params.yaml | sed 's/_params.yaml$//' | sort | \
  gum filter --placeholder "Select an agent to drive...") || exit 0
[[ -z ${agent_ns} ]] && exit 0

# --- Launch ---
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=True --remap cmd_vel:="/${agent_ns}/cmd_vel_joy"

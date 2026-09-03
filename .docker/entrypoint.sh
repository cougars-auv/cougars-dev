#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f "${OVERLAY_WS}/install/setup.bash" ] && source "${OVERLAY_WS}/install/setup.bash"

exec "$@"

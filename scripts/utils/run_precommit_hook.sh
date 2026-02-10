#!/bin/bash
# Created by Nelson Durrant, Jan 2026

hook_name="$1"
shift
args=()
files=()

for arg in "$@"; do
    if [[ "$arg" == -* ]]; then
        args+=("$arg")
    else
        files+=("src/${arg#packages/}")
    fi
done

docker exec cougars-ct /bin/bash -c \
    "source /opt/ros/humble/setup.bash \
    && cd coug_ws \
    && export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 \
    && $hook_name ${args[*]} \"\$@\"" -- "${files[@]}"

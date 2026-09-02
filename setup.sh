#!/bin/bash
set -e

use_sim_time="false"
version="latest"

args=()
while [ $# -gt 0 ]; do
  case "$1" in
    --hitl) use_sim_time="true"; shift ;;
    --version) version="$2"; shift 2 ;;
    --version=*) version="${1#*=}"; shift ;;
    *) args+=("$1"); shift ;;
  esac
done
set -- "${args[@]}"

if [ -z "$1" ] || [ -z "$2" ]; then
  echo "Usage: ./setup.sh <agent-ns> <base-station-ip> [--hitl] [--version <tag>]"
  exit 1
fi

cd "$(dirname "$0")"

agent_ns="$1"
ip="$2"

if [ ! -f "config/${agent_ns}_params.yaml" ]; then
  echo "Error: unknown agent '${agent_ns}'"
  exit 1
fi

vcs import ros2_ws/src < runtime.repos
vcs custom -n --git --args submodule update --init --recursive

for git_dir in $(find ros2_ws/src -maxdepth 2 -name .git -prune); do
  repo="$(dirname "${git_dir}")"
  git -C "${repo}" remote add base "git://${ip}/cougars-dev/${repo}" 2>/dev/null || \
    git -C "${repo}" remote set-url base "git://${ip}/cougars-dev/${repo}"
done
git remote add base "git://${ip}/cougars-dev" 2>/dev/null || \
  git remote set-url base "git://${ip}/cougars-dev"

cat > .env <<EOF
AGENT_NS=${agent_ns}
ZENOH_ROUTER_IP=${ip}
USE_SIM_TIME=${use_sim_time}
VERSION=${version}
EOF

sudo cp cougars.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable cougars.service

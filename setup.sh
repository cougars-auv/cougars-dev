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
  echo "Usage: ./setup.sh <base-station-ip> <agent-ns> [--hitl] [--version <tag>]"
  exit 1
fi

cd "$(dirname "$0")"

ip="$1"
agent_ns="$2"

if [ ! -f "config/${agent_ns}_params.yaml" ]; then
  echo "Error: unknown agent '${agent_ns}'"
  exit 1
fi

sed 's|git@github.com:|https://github.com/|g' cougars.repos | vcs import ros2_ws/src || true
vcs custom -n --git --args submodule update --init --recursive

for repo in ros2_ws/src/*/; do
  repo="${repo%/}"
  [ -d "${repo}/.git" ] || continue
  git -C "${repo}" remote add base "git://${ip}/cougars-dev/${repo}" 2>/dev/null || \
    git -C "${repo}" remote set-url base "git://${ip}/cougars-dev/${repo}"
done
git remote add base "git://${ip}/cougars-dev" 2>/dev/null || \
  git remote set-url base "git://${ip}/cougars-dev"

cat > .env <<EOF
VERSION=${version}
ZENOH_ROUTER_IP=${ip}
AUV_NS=${agent_ns}
USE_SIM_TIME=${use_sim_time}
EOF

sudo cp cougars.service /etc/systemd/system/
sudo systemctl enable --now cougars.service

#!/bin/bash
set -e

if [ -z "$1" ] || [ -z "$2" ]; then
  echo "Usage: ./setup.sh <base-station-ip> <agent-ns> [set-origin]"
  exit 1
fi

cd "$(dirname "$0")"
source "scripts/utils/common.sh"

ip="$1"
agent_ns="$2"
set_origin="${3:-true}"

if [ -z "${AGENTS[${agent_ns}]}" ]; then
  echo "Error: unknown agent '${agent_ns}'"
  exit 1
fi
auv_urdf="${AGENTS[${agent_ns}]}"

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
ZENOH_ROUTER_IP=${ip}
AUV_URDF=${auv_urdf}
AUV_NS=${agent_ns}
SET_ORIGIN=${set_origin}
EOF

sudo cp cougars.service /etc/systemd/system/
sudo systemctl enable --now cougars.service

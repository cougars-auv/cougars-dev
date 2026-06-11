#!/bin/bash
set -e

if [ -z "$1" ] || [ -z "$2" ]; then
  echo "Usage: ./setup.sh <base-station-ip> <agent-ns>"
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
ZENOH_ROUTER_IP=${ip}
AUV_NS=${agent_ns}
EOF

sudo cp cougars.service /etc/systemd/system/
sudo systemctl enable --now cougars.service

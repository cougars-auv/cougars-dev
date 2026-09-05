#!/bin/bash
# Copyright 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -e

use_sim_time="false"

args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
  --hitl)
    use_sim_time="true"
    shift
    ;;
  *)
    args+=("$1")
    shift
    ;;
  esac
done
set -- "${args[@]}"

if [[ -z $1 || -z $2 ]]; then
  echo "Usage: ./setup.sh <agent-ns> <base-station-ip> [--hitl]"
  exit 1
fi

cd "$(dirname "$0")"

agent_ns="$1"
ip="$2"

if [[ ! -f "config/${agent_ns}_params.yaml" ]]; then
  echo "Error: unknown agent '${agent_ns}'"
  exit 1
fi

cat >.env <<EOF
AGENT_NS=${agent_ns}
ZENOH_ROUTER_IP=${ip}
USE_SIM_TIME=${use_sim_time}
EOF

git remote add base "git://${ip}/cougars-dev" 2>/dev/null ||
  git remote set-url base "git://${ip}/cougars-dev"

./import.sh

sudo cp cougars.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable cougars.service

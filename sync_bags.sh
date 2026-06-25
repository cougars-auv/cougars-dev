#!/bin/bash
set -e

if [ -z "$1" ]; then
  echo "Usage: ./sync_bags.sh <agent-ns>"
  exit 1
fi

cd "$(dirname "$0")"

agent_ns="$1"

mkdir -p "bags/${agent_ns}"
rsync -avz --progress frostlab@"${agent_ns}".local:~/cougars-dev/bags/ "bags/${agent_ns}/"

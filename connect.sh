#!/bin/bash
set -e

if [[ -z $1 ]]; then
  echo "Usage: ./connect.sh <agent-ns>"
  exit 1
fi

cd "$(dirname "$0")"

agent_ns="$1"

ssh frostlab@"${agent_ns}".local \
  "tmux has-session -t couguv 2>/dev/null || tmuxp load -d ~/cougars-dev/config/tmuxp/couguv.yaml"
mosh frostlab@"${agent_ns}".local -- tmux attach -t couguv

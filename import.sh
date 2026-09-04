#!/bin/bash
set -e

cd "$(dirname "$0")"

if [[ ! -f .env ]]; then
  echo "Error: .env not found, run ./setup.sh first"
  exit 1
fi

ip="$(grep '^ZENOH_ROUTER_IP=' .env | cut -d= -f2-)"
if [[ -z ${ip} ]]; then
  echo "Error: ZENOH_ROUTER_IP not set in .env"
  exit 1
fi

# TODO: Add SSH key instructions for the CougUVs to support private repos
sed 's|git@github.com:|https://github.com/|g' runtime.repos | vcs import ros2_ws/src
vcs custom -n --git --args submodule update --init --recursive

while IFS= read -r git_dir; do
  repo="$(dirname "${git_dir}")"
  git -C "${repo}" remote add base "git://${ip}/cougars-dev/${repo}" 2>/dev/null || \
    git -C "${repo}" remote set-url base "git://${ip}/cougars-dev/${repo}"
done < <(find ros2_ws/src -name .git -prune)

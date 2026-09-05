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
  git -C "${repo}" remote add base "git://${ip}/cougars-dev/${repo}" 2>/dev/null ||
    git -C "${repo}" remote set-url base "git://${ip}/cougars-dev/${repo}"
done < <(find ros2_ws/src -name .git -prune)

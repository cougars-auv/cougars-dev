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

if [[ -z $1 ]]; then
  echo "Usage: ./sync_bags.sh <agent-ns>"
  exit 1
fi

cd "$(dirname "$0")"

agent_ns="$1"

mkdir -p "bags/${agent_ns}"
rsync -avzP frostlab@"${agent_ns}".local:~/cougars-dev/bags/ "bags/${agent_ns}/"

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

# Match UID/GID to local user
target_uid=$(stat -c '%u' "${CONFIG_DIR}")
target_gid=$(stat -c '%g' "${CONFIG_DIR}")

groupmod -o -g "${target_gid}" "${DOCKER_USER}"
usermod -o -u "${target_uid}" "${DOCKER_USER}"

chown "${DOCKER_USER}:${DOCKER_USER}" "/home/${DOCKER_USER}"
find "/home/${DOCKER_USER}" -maxdepth 1 -not -user "${DOCKER_USER}" \
  -exec chown -R "${DOCKER_USER}:${DOCKER_USER}" {} + 2>/dev/null || true

exec gosu "${DOCKER_USER}" "$@"

#!/bin/bash
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

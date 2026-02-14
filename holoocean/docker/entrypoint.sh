#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Runs automatic commands on Docker container startup
# - This script won't throw any errors, but it will crash immediately if any command fails
# - You can view the output of this script by running 'docker logs holoocean-ct'

set -e

# Fix permissions
DOCKER_USER=ue4
target_uid=$(stat -c '%u' /home/$DOCKER_USER/config)
target_gid=$(stat -c '%g' /home/$DOCKER_USER/config)

if [ ! -z "$target_gid" ]; then
    if [ "$target_gid" != "$(id -g $DOCKER_USER)" ]; then
        echo "Changing GID of $DOCKER_USER to $target_gid..."
        groupmod -o -g "$target_gid" $DOCKER_USER
    fi
fi
if [ ! -z "$target_uid" ]; then
    if [ "$target_uid" != "$(id -u $DOCKER_USER)" ]; then
        echo "Changing UID of $DOCKER_USER to $target_uid..."
        usermod -o -u "$target_uid" $DOCKER_USER
    fi
fi

find "/home/$DOCKER_USER" \
    -maxdepth 1 \
    -not -user "$DOCKER_USER" \
    -exec chown -R $DOCKER_USER:$DOCKER_USER {} + 2>/dev/null || true

touch /tmp/ready
exec "$@"

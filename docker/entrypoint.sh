#!/bin/bash
# Created by Nelson Durrant, Jan 2026
#
# Runs automatic commands on Docker container startup
# - This script won't throw any errors, but it will crash immediately if any command fails
# - You can view the output of this script by running 'docker logs cougars-ct'

set -e

# Fix permissions
DOCKER_USER=${DOCKER_USER}
target_uid=$(stat -c '%u' /home/$DOCKER_USER/ros2_ws/src)
target_gid=$(stat -c '%g' /home/$DOCKER_USER/ros2_ws/src)

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

# Install external ROS packages (vcs)
git config --system --add safe.directory "*"
if curl -s --head https://github.com | grep "200" > /dev/null; then
    echo "Network found. Updating vcs repositories..."
    gosu $DOCKER_USER vcs import ~/ros2_ws/src < ~/ros2_ws/src/cougars.repos
    gosu $DOCKER_USER vcs custom ~/ros2_ws/src --git --args submodule update --init --recursive
else
    echo "No network connection. Skipping vcs repository updates."
fi

# Start the SSH server
if [ -x /usr/sbin/sshd ]; then
    echo "Starting SSH server on port 2222..."
    mkdir -p /var/run/sshd
    ssh-keygen -A
    /usr/sbin/sshd
fi

touch /tmp/ready
exec "$@"

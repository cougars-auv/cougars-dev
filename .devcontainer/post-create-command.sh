#!/bin/bash
set -e

vcs import ros2_ws/src < cougars.repos
vcs import ros2_ws/src < dev.repos
vcs custom -n --git --args submodule update --init --recursive
find . -maxdepth 4 -name '.pre-commit-config.yaml' -execdir pre-commit install --install-hooks \;

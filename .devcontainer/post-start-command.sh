#!/bin/bash
set -e

git daemon --detach --reuseaddr --export-all --base-path=/home/frostlab-docker /home/frostlab-docker/cougars-dev

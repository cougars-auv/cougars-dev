#!/bin/bash
set -e

source ${OVERLAY_WS}/install/setup.bash

ros2 topic pub --once /origin sensor_msgs/msg/NavSatFix "{
  header: {frame_id: 'map'},
  status: {status: 0},
  latitude: 40.33940,
  longitude: -111.90721,
  altitude: 1412.0
}"

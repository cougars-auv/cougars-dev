#!/bin/bash
set -e

# --- Selection ---
scenario=$(gum choose --header "Choose a HoloOcean scenario:" \
  "CougUV" \
  "BlueROV2" \
  "WAM-V" \
  "CougUV Multi-Agent" \
  "BlueROV2 Multi-Agent" \
  "Mixed Multi-Agent")

case ${scenario} in
  "CougUV") params="couguv_openwater_params.yaml";;
  "BlueROV2") params="bluerov2_openwater_params.yaml";;
  "WAM-V") params="wamv_openwater_params.yaml";;
  "CougUV Multi-Agent") params="multi_couguv_openwater_params.yaml";;
  "BlueROV2 Multi-Agent") params="multi_bluerov2_openwater_params.yaml";;
  "Mixed Multi-Agent") params="multi_mixed_openwater_params.yaml";;
esac

# --- Launch ---
docker exec -it --user ue4 cougars-holoocean-ct /bin/bash -c \
  "source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/ue4/ros2_ws/install/setup.bash \
  && ros2 run holoocean_main holoocean_node --ros-args --params-file /home/ue4/config/holoocean/${params}"

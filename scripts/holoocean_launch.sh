#!/bin/bash
set -e

# --- Selection ---
scenario=$(gum choose --header "Choose a HoloOcean scenario:" "CougUV" "BlueROV2" "Multi-Agent")

case ${scenario} in
  "CougUV") params="couguv_holoocean_params.yaml";;
  "BlueROV2") params="bluerov2_holoocean_params.yaml";;
  "Multi-Agent") params="multi_couguv_holoocean_params.yaml";;
esac

# --- Launch ---
docker exec -it --user ue4 holoocean-ct /bin/bash -c \
  "source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/ue4/ros2_ws/install/setup.bash \
  && ros2 run holoocean_main holoocean_node --ros-args --params-file /home/ue4/config/holoocean/${params}"

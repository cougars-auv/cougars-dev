#!/bin/bash
BAG_DIR="${HOME}/cougars-dev/bags"

declare -A AGENTS=(
  ["bluerov2"]="bluerov2.urdf.xacro"
  ["turtlmap"]="turtlmap.urdf.xacro"
  ["aquaslam"]="aquaslam.urdf.xacro"
  ["aquaslam_wt"]="aquaslam_wt.urdf.xacro"
  ["coug2_dvl"]="couguv.urdf.xacro"
  ["coug2_ekf"]="couguv.urdf.xacro"
  ["blue0sim"]="bluerov2_holoocean.urdf.xacro"
  ["coug0sim"]="couguv_holoocean.urdf.xacro"
  ["coug1sim"]="couguv_holoocean.urdf.xacro"
  ["coug2sim"]="couguv_holoocean.urdf.xacro"
)

agents_yaml() {
  local out="" sep="" ns
  for ns in "$@"; do
    if [ -z "${AGENTS[${ns}]}" ]; then
      return 1
    fi
    out+="${sep}[${ns}, ${AGENTS[${ns}]}]"
    sep=", "
  done
  printf '[%s]' "${out}"
}

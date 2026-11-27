#!/usr/bin/env bash
# Start PX4 SITL in Gazebo with the x500_lidar_3d model, talking to ROS 2 over Zenoh.
#
# PX4 spawns models only from its own Tools/simulation/gz/models folder, so the model
# shipped in this package is symlinked there first. `make px4_sitl gz_<model>` forces
# PX4_SIM_MODEL itself, so the PX4 binary is started directly (same command the make
# target runs) with the x500 airframe (4001) and our model.
#
# Env overrides: PX4_DIR (default ~/PX4-Autopilot), PX4_GZ_WORLD (default walls),
# HEADLESS=1 (no Gazebo GUI), plus any other PX4_* / PX4_GZ_* variable.
# Needs ROS 2 sourced (vendored Gazebo libraries) and rmw_zenohd running.
set -euo pipefail

PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MODEL=x500_lidar_3d
BUILD="$PX4_DIR/build/px4_sitl_default"

if ! command -v gz >/dev/null; then
  echo "gz not found: source your ROS 2 workspace first" >&2
  exit 1
fi
if [ ! -x "$BUILD/bin/px4" ]; then
  echo "PX4 SITL not built: run 'make px4_sitl' in $PX4_DIR" >&2
  exit 1
fi

# The model lives in the source tree; prefer it over an installed copy so edits apply.
MODEL_SRC="$HERE/models/$MODEL"
ln -sfn "$MODEL_SRC" "$PX4_DIR/Tools/simulation/gz/models/$MODEL"

cd "$BUILD/src/modules/simulation/gz_bridge"
exec env \
  PX4_ROS2_MIDDLEWARE="${PX4_ROS2_MIDDLEWARE:-zenoh}" \
  PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4001}" \
  PX4_SIM_MODEL="gz_$MODEL" \
  PX4_GZ_WORLD="${PX4_GZ_WORLD:-walls}" \
  GZ_IP="${GZ_IP:-127.0.0.1}" \
  "$BUILD/bin/px4"

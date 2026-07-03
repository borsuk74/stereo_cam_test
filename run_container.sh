#!/usr/bin/env bash
# Starts the kalibr ROS1 container, mounting a host directory for rosbag output
# so recordings survive container restart/removal (container itself is ephemeral,
# started with --rm).
#
# Host processes (host_dual_cam_stream.py, host_imu_stream.py) must already be
# running so the container can connect to them at 172.17.0.1 (docker0 gateway).
#
# Inside the container, record with a path under /root/bags, e.g.:
#   rosbag record -O /root/bags/calibration.bag /cam0/image_raw /cam1/image_raw /imu

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAGS_DIR="$REPO_DIR/bags"
mkdir -p "$BAGS_DIR"

docker run --rm -it \
  -v "$BAGS_DIR:/root/bags" \
  kalibr:ros1-20.04-arm64_sn

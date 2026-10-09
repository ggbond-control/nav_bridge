#!/usr/bin/env bash
# Build vendored Cyvet dependencies and nav_bridge in the same driver workspace.
set -eo pipefail
export MAKEFLAGS="-j2 -l2"
cyvet_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cyvet_workspace=${1:-$(cd "$cyvet_source/../.." && pwd)}
source /opt/ros/jazzy/setup.bash
colcon --log-base "$cyvet_workspace/log" build \
  --base-paths "$cyvet_source/third_party/uniubi/uniubi_robot_msgs/ros2" \
               "$cyvet_source/third_party/uniubi/uniubi_motion_client" \
  --build-base "$cyvet_workspace/build" --install-base "$cyvet_workspace/install" \
  --parallel-workers 2 --symlink-install --cmake-clean-cache --cmake-args -DBUILD_TESTING=OFF
source "$cyvet_workspace/install/local_setup.bash"
colcon --log-base "$cyvet_workspace/log" build \
  --base-paths "$cyvet_source" --packages-select nav_bridge \
  --build-base "$cyvet_workspace/build" --install-base "$cyvet_workspace/install" \
  --symlink-install --cmake-clean-cache --cmake-args -DNAV_BRIDGE_BUILD_X30=OFF -DNAV_BRIDGE_BUILD_D1_MAX=OFF \
              -DNAV_BRIDGE_BUILD_CYVET=ON -DBUILD_TESTING="${CYVET_BUILD_TESTING:-OFF}"

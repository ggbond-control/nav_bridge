#!/usr/bin/env bash
set -e
source /opt/runtime/env.bash
source /opt/ros/humble/setup.bash
source /home/robot/Workspace/driver_ws/install/cloud_merge/share/cloud_merge/local_setup.bash
cloud_merge_prefix=/home/robot/Workspace/driver_ws/install/cloud_merge
export AMENT_PREFIX_PATH="${cloud_merge_prefix}:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="${cloud_merge_prefix}:${CMAKE_PREFIX_PATH:-}"
exec /opt/ros/humble/bin/ros2 launch cloud_merge cloud_merge.launch.py use_rviz:=false

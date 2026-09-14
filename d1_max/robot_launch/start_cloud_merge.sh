#!/usr/bin/env bash
set -e
source /opt/runtime/env.bash
source /home/robot/Workspace/driver_ws/install/setup.bash
exec ros2 launch cloud_merge cloud_merge.launch.py use_rviz:=false

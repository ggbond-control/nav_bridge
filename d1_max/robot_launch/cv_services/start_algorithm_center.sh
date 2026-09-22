#!/usr/bin/env bash
set -euo pipefail

exec /usr/bin/setpriv --reuid=robot --regid=robot --init-groups \
  /usr/bin/env \
    HOME=/home/robot \
    USER=robot \
    LOGNAME=robot \
    XDG_CACHE_HOME=/home/robot/.cache \
    XDG_CONFIG_HOME=/home/robot/.config \
    PATH=/home/robot/miniconda3/envs/cable/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    /home/robot/CV/bin/run_algorithm_center.sh

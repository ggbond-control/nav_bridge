#!/bin/sh
set -eu
if [ "$(id -u)" -eq 0 ]; then
    exec systemctl stop nx-net-proxy.service
fi
exec sudo systemctl stop nx-net-proxy.service

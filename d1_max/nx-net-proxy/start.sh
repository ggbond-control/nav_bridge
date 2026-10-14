#!/bin/sh
# Both this script and service commands control the same systemd process.
set -eu
if [ "$(id -u)" -eq 0 ]; then
    exec systemctl start nx-net-proxy.service
fi
exec sudo systemctl start nx-net-proxy.service

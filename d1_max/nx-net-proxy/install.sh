#!/bin/sh
# Run as cat on D1M-B after copying this directory to the target path.
set -eu
NX_PROXY_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ "$NX_PROXY_DIR" != /home/cat/Scripts/nx-net-proxy ] || [ "$(id -un)" != cat ]; then
    echo 'Run as cat: sh /home/cat/Scripts/nx-net-proxy/install.sh' >&2
    exit 1
fi
sudo -v
if [ ! -f "$NX_PROXY_DIR/lib/proxy/__main__.py" ]; then
    /usr/bin/python3 -m zipfile -e \
        "$NX_PROXY_DIR/proxy.py-2.4.10-py3-none-any.whl" "$NX_PROXY_DIR/lib"
fi
chmod 700 "$NX_PROXY_DIR/run.sh" "$NX_PROXY_DIR/start.sh" "$NX_PROXY_DIR/stop.sh"
# Stop only a matching legacy PID, never an unrelated process.
if [ -f "$NX_PROXY_DIR/proxy.pid" ]; then
    sh "$NX_PROXY_DIR/legacy/stop.sh"
fi
sudo systemd-analyze verify "$NX_PROXY_DIR/nx-net-proxy.service"
sudo install -m 644 "$NX_PROXY_DIR/nx-net-proxy.service" /etc/systemd/system/nx-net-proxy.service
sudo systemctl daemon-reload
sudo systemctl enable nx-net-proxy.service
sudo systemctl restart nx-net-proxy.service
systemctl is-enabled nx-net-proxy.service
systemctl is-active nx-net-proxy.service

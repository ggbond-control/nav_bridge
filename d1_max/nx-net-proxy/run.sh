#!/bin/sh
# Foreground entry point for systemd; use start.sh for manual operation.
set -eu
NX_PROXY_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
export PYTHONPATH="$NX_PROXY_DIR/lib"
exec /usr/bin/python3 -m proxy \
    --hostname 192.168.168.111 --port 18080 \
    --threaded --num-workers 1 --num-acceptors 1 \
    --plugins proxy.plugin.FilterByClientIpPlugin \
    --filtered-client-ips-mode whitelist \
    --filtered-client-ips 192.168.168.100,192.168.168.111 \
    --data-dir "$NX_PROXY_DIR/data" --cache-dir "$NX_PROXY_DIR/cache"

#!/bin/sh
set -eu
NX_PROXY_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ -s "$NX_PROXY_DIR/proxy.pid" ] && kill -0 "$(cat "$NX_PROXY_DIR/proxy.pid")" 2>/dev/null; then
    echo "Proxy already running at http://192.168.168.111:18080"
    exit 0
fi
nohup env PYTHONPATH="$NX_PROXY_DIR/lib" python3 -m proxy \
    --hostname 192.168.168.111 --port 18080 \
    --threaded --num-workers 1 --num-acceptors 1 \
    --plugins proxy.plugin.FilterByClientIpPlugin \
    --filtered-client-ips-mode whitelist \
    --filtered-client-ips 192.168.168.100,192.168.168.111 \
    --data-dir "$NX_PROXY_DIR/data" --cache-dir "$NX_PROXY_DIR/cache" \
    --pid-file "$NX_PROXY_DIR/proxy.pid" \
    > "$NX_PROXY_DIR/proxy.log" 2>&1 < /dev/null &
python3 - "$NX_PROXY_DIR" <<'PY'
import pathlib, socket, sys, time
directory = pathlib.Path(sys.argv[1])
for _ in range(25):
    try:
        with socket.create_connection(('192.168.168.111', 18080), timeout=0.5):
            print('Proxy ready at http://192.168.168.111:18080')
            break
    except OSError:
        time.sleep(0.2)
else:
    print((directory / 'proxy.log').read_text(), file=sys.stderr)
    sys.exit(1)
PY

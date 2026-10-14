#!/bin/sh
set -eu
NX_PROXY_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
python3 - "$NX_PROXY_DIR" <<'PY'
import os, pathlib, signal, sys, time
directory = pathlib.Path(sys.argv[1])
pid_file = directory / 'proxy.pid'
if not pid_file.exists():
    print('Proxy already stopped')
    sys.exit(0)
pid = int(pid_file.read_text().strip())
try:
    argv = pathlib.Path('/proc/{}/cmdline'.format(pid)).read_bytes().split(b'\0')
except FileNotFoundError:
    pid_file.unlink()
    print('Proxy already stopped')
    sys.exit(0)
if b'proxy' not in argv or os.fsencode(str(directory / 'data')) not in argv:
    sys.exit('PID does not belong to this proxy; no signal sent')
os.kill(pid, signal.SIGTERM)
for _ in range(50):
    if not pid_file.exists():
        print('Proxy stopped')
        break
    time.sleep(0.1)
else:
    sys.exit('Proxy is still stopping; check proxy.log')
PY

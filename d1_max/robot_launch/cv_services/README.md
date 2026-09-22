# CV Services Managed By `robot-launch`

This backup installs two independent `robot-launch` eggs on the D1 Max NX host:

- `algorithm-center`: the Java routing service on TCP port `25682`.
- `cable-damage-detect`: the cable-damage inference HTTP service on TCP port `8000`.

`robot-launch` is started by root. The two `start_*.sh` wrappers use `setpriv` to
run the long-lived application as `robot`; this preserves ownership of
`/home/robot/CV/java/logs`, uploads, model caches, and generated results. They
also explicitly set `HOME`, `XDG_CACHE_HOME`, and `XDG_CONFIG_HOME` to the
`robot` home directory. `setpriv` changes process identity but does not clear
root's inherited environment; without these values, Gunicorn and Ultralytics
may try to write under `/root` and the detector will remain unhealthy.

The Java package's original `app.sh` must not be used as the egg command. It
backgrounds Java with `nohup` and returns immediately, so `robot-launch` cannot
monitor, stop, or restart the actual JVM. `run_algorithm_center.sh` instead uses
`exec java` in the foreground.

The detector is launched with a single Gunicorn worker and four threads. A single
worker avoids loading an additional copy of the YOLO model into Orin memory.

## Deployment

1. Copy all four shell scripts to `/home/robot/CV/bin/` and mark them executable.
2. Copy `robot_egg_entries.yaml` below the existing `eggs:` mapping in
   `/home/robot/.robot_egg_launch.yaml`; IDs 18 and 19 must remain unique.
3. Stop any manually started Java/Python instances using ports 25682 and 8000.
4. Restart `robot-launch.service` so it reloads the YAML configuration.
5. Verify:

```bash
robot-launch egg algorithm-center
robot-launch egg cable-damage-detect
curl -fsS http://127.0.0.1:8000/health
ss -ltnp | grep -E ':(8000|25682)\\b'
```

The Java application's root endpoint currently returns HTTP 404 and does not
expose `/health` or `/actuator/health`; verify it by its managed process and
listening port until the application provides an explicit health endpoint.

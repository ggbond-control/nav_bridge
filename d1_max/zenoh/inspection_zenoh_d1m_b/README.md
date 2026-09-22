# D1M-B Inspection Zenoh Service

This directory is the deployment backup for `inspection-zenoh.service` on:

```text
D1M-B: cat@10.0.40.195
```

The service runs the Jazzy `rmw_zenohd` router as user `cat`. It listens on
TCP port `7447` and connects to the D1 navigation host router at
`192.168.168.100:7447`.

## Domain

At deployment, D1M-B's interactive `~/.zshrc` resolved these values:

```text
ROS_DOMAIN_ID=28
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_ROUTER_CONFIG_URI=/home/cat/Workspace/zenoh_ws/router_config.json5
ZENOH_SESSION_CONFIG_URI=/home/cat/Workspace/zenoh_ws/session_config.json5
```

Systemd does not source an interactive shell configuration. The unit therefore
sets the same values explicitly. If D1M-B's `~/.zshrc` later changes its ROS
domain, update `Environment=ROS_DOMAIN_ID=...` in the unit to the new value and
restart the service.

## Install Or Restore

```bash
sudo install -o root -g root -m 0644 inspection-zenoh.service \
  /etc/systemd/system/inspection-zenoh.service
sudo install -o cat -g cat -m 0644 router_config.json5 \
  /home/cat/Workspace/zenoh_ws/router_config.json5
sudo install -o cat -g cat -m 0644 session_config.json5 \
  /home/cat/Workspace/zenoh_ws/session_config.json5
sudo systemctl daemon-reload
sudo systemctl enable --now inspection-zenoh.service
```

## Verify

```bash
systemctl status inspection-zenoh.service --no-pager
systemctl is-enabled inspection-zenoh.service
ss -ltnp | grep ':7447'
sudo journalctl -u inspection-zenoh.service -n 100 --no-pager
```

The router process must be owned by `cat`; do not additionally start
`ros2 run rmw_zenoh_cpp rmw_zenohd` in an SSH shell, because it would conflict
with the service on port `7447`.

## Deployment Result: 2026-09-22

The service was installed, enabled, and verified active on D1M-B. Its router
listens on `0.0.0.0:7447` and its process environment has
`ROS_DOMAIN_ID=28`, matching the value resolved from D1M-B's `~/.zshrc`.

At verification time, D1M-B had only `wlan0` (`10.0.40.195/24`) and a default
route through `10.0.40.1`. It had no interface or route in
`192.168.168.0/24`, so its configured NX connection
`192.168.168.100:7447` remained in `SYN-SENT`. By comparison, D1M-A reaches
that NX address over `eth0` from `192.168.168.111`.

The systemd deployment is correct, but D1M-B cannot exchange ROS traffic with
the NX router until its own NX network is connected and routed, or until
`router_config.json5` is updated with D1M-B's actual reachable NX endpoint.

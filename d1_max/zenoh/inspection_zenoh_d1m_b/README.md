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
sudo install -d -o root -g root -m 0755 \
  /etc/systemd/system/inspection-zenoh.service.d
sudo install -o root -g root -m 0644 inspection-zenoh.service.d/priority.conf \
  /etc/systemd/system/inspection-zenoh.service.d/priority.conf
sudo install -o cat -g cat -m 0644 router_config.json5 \
  /home/cat/Workspace/zenoh_ws/router_config.json5
sudo install -o cat -g cat -m 0644 session_config.json5 \
  /home/cat/Workspace/zenoh_ws/session_config.json5
sudo systemctl daemon-reload
sudo systemctl enable inspection-zenoh.service
sudo systemctl restart inspection-zenoh.service
```

## Verify

```bash
systemctl status inspection-zenoh.service --no-pager
systemctl is-enabled inspection-zenoh.service
systemctl show inspection-zenoh.service -p Nice -p CPUWeight
ss -ltnp | grep ':7447'
sudo journalctl -u inspection-zenoh.service -n 100 --no-pager
```

The router process must be owned by `cat`; do not additionally start
`ros2 run rmw_zenoh_cpp rmw_zenohd` in an SSH shell, because it would conflict
with the service on port `7447`.

The priority drop-in sets `Nice=-5` and `CPUWeight=600`. On D1M-B, this keeps
Zenoh below localization/navigation (`Nice=-10`) but above the gimbal
(`Nice=5`). Restarting Zenoh briefly interrupts ROS connections.

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

## Session Configuration Fix: 2026-10-15

The deployment copy `session_config.json5` now matches the validated D1M-B
configuration: multicast scouting stays enabled and uses only `eth1`
(`192.168.168.111/24` at verification). This avoids creating discovery sockets
on transient Wi-Fi/cellular addresses, which caused `Unable to send Hello`
after an address disappeared. The router configuration and systemd files are
unchanged.

The original session configuration is preserved byte-for-byte in
[`originals/20261015_before_eth1/session_config.json5`](originals/20261015_before_eth1/session_config.json5).
Validation results and limitations are recorded in
[`reports/20261015_eth1.md`](reports/20261015_eth1.md).
The original device-side configuration and full evidence are under
`/home/cat/Workspace/zenoh_ws/validation/20261015_eth1`.

Validation retained all 76 nodes and 328 topics and passed topic, service,
action and QoS interoperability checks with both original and updated clients.
No business process or router was restarted. Observed brief message gaps also
occurred in the original-only control; the checks do not guarantee zero jitter.
Wi-Fi/cellular multicast discovery is excluded by this configuration. Actual
network switching was not tested.

For this session-only fix, install only the session file on D1M-B:

```bash
sudo install -o cat -g cat -m 0644 session_config.json5 \
  /home/cat/Workspace/zenoh_ws/session_config.json5
```

Existing ROS processes retain their startup configuration. They load this file
on their next restart; existing processes may continue to log Hello errors
until then. A router restart is not required for this session-only update.

To restore the original session configuration, run from this backup directory
on D1M-B:

```bash
sudo install -o cat -g cat -m 0644 \
  originals/20261015_before_eth1/session_config.json5 \
  /home/cat/Workspace/zenoh_ws/session_config.json5
```

Rollback likewise takes effect when ROS processes next start. This backup
update itself does not restart any device processes.

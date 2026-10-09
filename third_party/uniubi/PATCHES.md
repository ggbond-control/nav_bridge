# Local changes to the pinned upstream sources

Sources and exact commits are recorded in versions.json. LICENSE, NOTICE and
third-party notices are retained alongside the source. No motion SDK binaries are used.

uniubi_motion_client/src/motion_high_level_client.cpp:
- Event reader uses BEST_EFFORT/VOLATILE, matching the documented host contract
  while also accepting the RELIABLE publisher observed on Cyvet-V1.00.000.
  Upstream's default RELIABLE reader cannot match BEST_EFFORT publishers.
- Synchronous RPC and asynchronous lease renewal replies must match the requested device_id. A successful
  transport reply from another robot must not authorize an action.

The vendored directory has COLCON_IGNORE to keep X30/D1 builds independent.
Build the two dependency packages with explicit --base-paths before enabling Cyvet.

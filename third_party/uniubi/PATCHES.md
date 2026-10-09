# Local changes to the pinned upstream sources

Sources and exact commits are recorded in versions.json. LICENSE, NOTICE and
third-party notices are retained alongside the source. No motion SDK binaries are used.

uniubi_motion_client/src/motion_high_level_client.cpp:
- Event reader uses BEST_EFFORT/VOLATILE, matching the documented host contract
  while also accepting the RELIABLE publisher observed on Cyvet-V1.00.000.
  Upstream's default RELIABLE reader cannot match BEST_EFFORT publishers.
- Synchronous RPC and asynchronous lease renewal replies must match the requested device_id. A successful
  transport reply from another robot must not authorize an action.
- RPC deadline expiry has its own SystemRpcTimeout exception and appended
  kRpcTimeout enum value. Existing enum numbers are unchanged; the backend can
  distinguish a deadline from rejection, wrong identity, malformed replies or
  interruption, which must never trigger velocity retry.
- Lease renewal response wait is bounded by lease/10 (200 ms minimum, 3 s
  maximum) and the current expiry deadline. A 5 s lease retries a missed reply
  after 500 ms instead of consuming 3 s near expiry. It never extends the
  lease without a valid reply or control action.

The vendored directory has COLCON_IGNORE to keep X30/D1 builds independent.
Build the two dependency packages with explicit --base-paths before enabling Cyvet.

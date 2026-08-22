# D1 Max — Custom Control, SLAM & Mission Development

Engineering notes and build plans for developing third-party control software,
SLAM/mapping, and mission tooling on the AgiBot D1 Max, targeting Linux, Android
and iOS clients.

These documents are derived from the contents of this repository:
the `RobotSDK-0.1.1` headers and examples, `docs/protocol/Protocol-1.2.0.pdf`,
the Sphinx manual under `docs/source/`, and inspection of the shipped
`librobot_sdk.so` binaries.

## Contents

| Doc | What it covers |
|---|---|
| [01 — Platform architecture](01-platform-architecture.md) | The two-computer split, network map, what the SDK does and does not give you |
| [02 — Wire protocol](02-wire-protocol.md) | The `ZSKJ` framing + JSON message reference, for reimplementing the client in any language |
| [03 — SLAM & mapping](03-slam-mapping-plan.md) | Sensor inventory, the missing-odometry problem, and a concrete LIO stack plan |
| [04 — Clients (Linux / Android / iOS)](04-clients-linux-android-ios.md) | Why the shipped `.so` cannot be used on mobile, and the shared-core architecture that replaces it |
| [05 — Mission system](05-mission-system.md) | Mission schema, executor placement, waypoint following, safety interlocks |

A tested, dependency-free reference implementation of the framing codec lives at
[`tools/d1max_proto.py`](../../tools/d1max_proto.py).

## TL;DR

**The single most important fact:** the D1 Max is two computers with two
completely different interfaces, and they are not integrated for you.

- **RK3588** — motion control. Speaks a simple, *fully documented* binary+JSON
  protocol on UDP `8082` / WebSocket `8081`. `librobot_sdk.so` is a thin C++
  wrapper over it. Also serves RTSP video on `8554`.
- **Orin NX** — perception. Speaks ROS 2 Humble over the Zenoh RMW
  (`ROS_DOMAIN_ID=24`, router at `192.168.168.100:7447`). LiDAR, IMU,
  ultrasonics, RTK, cameras.

There is **no odometry topic, no `/tf`, no `/map`, and no navigation API** in
anything shipped here. The "SLAM mapping and navigation API" listed as an
optional V0.1.0 feature in `docs/source/2.2SDK软件服务接口列表.md` is not present
in this SDK — the headers contain no such functions. Assume you are building
the entire autonomy stack yourself.

**The good news:** the wire protocol is documented and trivial (16-byte header +
JSON body). You do not need the shipped `.so` at all — and on Android and iOS you
*cannot* use it. Reimplementing the client is roughly 300 lines per language, and
that reimplementation is what unblocks mobile.

**The bridge that makes SLAM work:** the control board publishes a 50 Hz motion
estimate (position, quaternion, body/world velocities, nanosecond timestamps) as
protocol message `1102`. Republishing that as `nav_msgs/Odometry` on the Orin NX
is the piece that lets an off-the-shelf LiDAR-inertial SLAM stack run on this
robot. See [doc 03](03-slam-mapping-plan.md).

## Suggested build order

1. **Protocol core + safety layer** — framing, heartbeat, watchdog, control-ownership
   handling. Everything else depends on it.
2. **Odometry bridge** on the Orin NX — protocol `1102` → `/odom` + `/tf`.
3. **SLAM** — FAST-LIO2 or Point-LIO on `/front_lidar` + `/front_lidar/imu`.
4. **Mission executor** on the Orin NX, not on the phone.
5. **Clients** — Linux first (fastest iteration), then a shared-core mobile app.

# 03 — SLAM & Mapping Plan

## What you have

All perception is ROS 2 Humble on the Orin NX, reached over the Zenoh RMW.

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/front_lidar` | `sensor_msgs/PointCloud2` | 10 Hz | 96-line, 360°×90°, 120 m range. QoS **best_effort** |
| `/rear_lidar` | `sensor_msgs/PointCloud2` | 10 Hz | same |
| `/front_lidar/imu` | `sensor_msgs/Imu` | — | **built into the LiDAR** |
| `/rear_lidar/imu` | `sensor_msgs/Imu` | — | |
| `/imu_driver/imu_central` | `sensor_msgs/Imu` | — | automotive-grade LUA300C |
| `/uss_driver/uss_left/range` | `sensor_msgs/Range` | 10 Hz | 0–4 m |
| `/uss_driver/uss_right/range` | `sensor_msgs/Range` | 10 Hz | 0–4 m |
| `/front_camera/image_compressed` | compressed image | — | from RK3588 |
| `/rear_camera/image_compressed` | compressed image | — | |
| `/rtk_pvh` | custom | — | GNSS/RTK position + heading, optional hardware |

Plus, off the ROS graph entirely:

- RTSP H.264 video on `rtsp://<rk3588>:8554/{front,back}` — low latency via
  `gst-launch-1.0 rtspsrc ... latency=0`.
- Protocol message `1102` at 50 Hz over UDP: position, orientation, body and world
  velocities, nanosecond timestamps.

## What you don't have

**No `/odom`. No `/tf`. No `/map`. No planner. No localisation.**

This is the gap that defines the project. The "SLAM 建图导航功能及 API 接口 (选配)"
listed at V0.1.0 in `docs/source/2.2` is an optional module that is not in this
repository, and the SDK headers contain no corresponding functions.

Verify what is actually running on your unit before building — the answer changes
the plan considerably:

```bash
ssh robot@192.168.168.100          # password: 1
ros2 topic list
ros2 node list
robot-launch help                  # on-device process manager; lists managed nodes
```

If a vendor SLAM node is present, prefer integrating with it. The plan below
assumes it is not.

## The critical enabler: synthesising odometry

A LiDAR-inertial SLAM stack needs a motion prior. On wheeled robots that is wheel
encoders; a legged robot has no equivalent, and none is published.

But protocol message `1102` **is** that prior. It carries, at 50 Hz:

```
quat[4]         orientation (w, x, y, z)
position[3]     metres, world frame
v_world[3]      m/s, world frame
v_body[3]       m/s, body frame
omega_world[3]  rad/s
omega_body[3]   rad/s
time_stamp      nanoseconds
```

That is a complete `nav_msgs/Odometry` message. The control board's state estimator
is already fusing leg kinematics with the ICM42688 IMU to produce it.

**So the first thing to build is an odometry bridge**: a small node on the Orin NX
that opens a UDP socket to the RK3588 at `192.168.168.168:8082`, enables sensor
`30` via message `1008`, and republishes `1102` as `/odom` plus the
`odom → base_link` transform.

```
   RK3588                       Orin NX
 ┌──────────┐   UDP 8082      ┌─────────────────────┐
 │  1102    │────────────────▶│  d1max_odom_bridge  │
 │  50 Hz   │   ZSKJ+JSON     │                     │
 └──────────┘                 │  → /odom            │
                              │  → /tf odom→base_link
                              └──────────┬──────────┘
                                         │
   /front_lidar (10 Hz) ─────────────────┤
   /front_lidar/imu     ─────────────────┤
                                         ▼
                              ┌─────────────────────┐
                              │  FAST-LIO2 / Point-LIO │
                              │  → /Odometry (map)  │
                              │  → /cloud_registered│
                              └──────────┬──────────┘
                                         ▼
                              PCD map · 2-D occupancy grid
```

This node is small — a few hundred lines — and it unlocks the entire off-the-shelf
ROS 2 SLAM ecosystem. Build it first.

**Caveats to handle in the bridge:**

- **Clock domains.** `1102`'s `time_stamp` is nanoseconds from the RK3588; ROS
  message stamps come from the Orin NX. The two boards are not obviously
  synchronised, and `SystemClockSanityError` exists as a fault code precisely
  because clock jumps happen. Estimate the offset on connect (the `1001` heartbeat
  echo gives you round-trip time), track drift, and expose the residual as a
  diagnostic. Do not silently restamp with `now()` — a hidden, varying latency
  between odometry and LiDAR will quietly wreck map quality in a way that is very
  hard to diagnose later.
- **UDP loss.** Datagrams drop. Detect gaps and mark the covariance accordingly
  rather than interpolating over them.
- **Drift.** This is dead reckoning on a legged platform. Yaw drift and z-drift on
  slopes and stairs will be significant. It is a *prior*, not ground truth —
  publish generous, honest covariance and let the SLAM back-end do the correcting.
- **Frame conventions.** Confirm the `1102` world frame against ROS REP-103
  (x-forward, y-left, z-up, right-handed) empirically. The `quat[4]` ordering in
  `MotionData` is documented `[w, x, y, z]`; ROS uses `[x, y, z, w]`. Getting this
  wrong produces a map that looks *almost* right, which is worse than one that
  looks obviously broken.

## Recommended SLAM stack

**Front-end: FAST-LIO2 or Point-LIO on `/front_lidar` + `/front_lidar/imu`.**

The LiDAR has an IMU built into it. That is exactly the tightly-coupled
LiDAR-inertial pairing these algorithms are designed for, with the extrinsic
between the two being small, fixed, and given by the vendor. It removes the hardest
calibration problem in the stack.

Why this family over the alternatives:

- **vs. Cartographer / SLAM Toolbox** — those want 2-D scans or a strong odometry
  source. You have neither natively, and flattening a 96-line cloud to fake a
  `/scan` throws away the vertical information you need for stairs and 80 cm
  obstacles.
- **vs. LIO-SAM** — LIO-SAM wants a 9-axis IMU with a reliable magnetometer and is
  fussier about initialisation. FAST-LIO2 is more robust to a legged platform's
  jerky, high-frequency motion.
- **Point-LIO** specifically handles high-rate, aggressive motion better than
  FAST-LIO2, which is a real consideration for a trotting quadruped. Try FAST-LIO2
  first for its maturity; switch if gait-induced vibration degrades the result.

**Back-end for large or loopy environments:** add a pose-graph layer with loop
closure. FAST-LIO2 alone drifts without loop closure over long traverses.

**Extrinsics** come from `docs/source/2.10` and the URDF. Front LiDAR sits at
(404.3, 0, −37.7) mm from BASE. Publish the TF tree from
`urdf/max_description/urdf/max.urdf` via `robot_state_publisher` so your
extrinsics and your model cannot drift apart. Feed `JointStateData` (enable
sensor `50` via message `1008`) into it for a live articulated model.

### Should you use both LiDARs?

Not at first. Front and rear give near-complete spherical coverage, which is
excellent for mapping — but doubles bandwidth and CPU, and multi-LiDAR extrinsic
calibration is genuinely hard. Get a good map from the front unit alone, then
evaluate whether the rear unit improves it enough to justify the cost. The rear
unit's clearest early win is obstacle detection while reversing, not mapping.

### Mapping outputs

- **3-D**: PCD point-cloud map from the LIO front-end. Ground truth for the map.
- **2-D**: project to an occupancy grid for planning and for a usable mobile UI.
  Filter by height band — around 0.15 m to 1.2 m above the estimated ground plane
  clears both the floor and overhangs the robot passes under.
- **Traversability**, eventually: this is a legged robot. It climbs 25 cm steps
  continuously and crosses 80 cm obstacles. A binary occupancy grid throws that
  capability away. A slope-and-step-height cost layer over the 3-D map is where
  this platform's advantage actually lives — but treat it as phase 3, after
  something simpler works end to end.

### Global referencing with RTK

`/rtk_pvh` gives lat/lon/altitude plus dual-antenna heading, with per-axis standard
deviations, `pos_type` (50 = integer fix = centimetre-grade) and solution age.
Two uses:

1. Anchor the SLAM map to a global frame so missions are portable across sessions
   and units.
2. Constrain drift on long outdoor traverses via loose coupling into the pose graph.

Requires an external SMA antenna, a Sixents service account (AK/AS in
`/ota/sixents_config.ini` on the Orin NX), and 4G connectivity for corrections.
It is a **custom message type, not `sensor_msgs/NavSatFix`**, so a converter node
is needed. Gate all of this on `pos_type` and the reported standard deviations —
degraded RTK is far more damaging to a pose graph than no RTK, because it is
confidently wrong.

## Connecting a development laptop

From `docs/source/5.3`. This is your day-one setup for RViz, `ros2 bag`, and
running your own nodes off-robot:

```bash
sudo apt-get install ros-humble-rmw-zenoh-cpp

# Edit the Zenoh router config to point at the robot:
#   /opt/ros/humble/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5
#   connect: { endpoints: ["tcp/192.168.168.100:7447"] }

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rmw_zenoh_cpp rmw_zenohd

ros2 daemon stop && ros2 daemon start
```

Set your laptop's IP to `192.168.168.x`, `x ∉ {100, 168, 255}`.

**Record bags early and often.** Two 96-line clouds at 10 Hz plus IMU plus your
synthesised odometry is a large but very tractable dataset, and being able to
iterate on SLAM tuning against a recorded traverse — instead of walking a 41 kg
robot around every time — is the single biggest multiplier on development speed
here. Capture a few good traverses (loop closure, stairs, outdoor, cluttered
indoor) before writing any tuning code.

## Where to run what

| Component | Where | Why |
|---|---|---|
| Odometry bridge | Orin NX | Must not depend on Wi-Fi |
| SLAM front-end | Orin NX | 157 TOPS, and the LiDAR data never has to leave the robot |
| Map storage | Orin NX + synced off-board | OTA/reflash wipes the Orin — back it up |
| Mission executor | Orin NX | Must survive client disconnection ([doc 05](05-mission-system.md)) |
| Map viewing / editing | Client | |
| Mission authoring | Client | |
| Teleop | Client | Direct to RK3588 |

The rule: **anything whose failure endangers the robot runs on the robot.** A phone
that loses signal must degrade to "robot continues or safely stops", never to
"robot is uncommanded mid-stride".

Bandwidth reinforces this. Two 96-line clouds at 10 Hz is far too much for a phone
over Wi-Fi. Clients get derived products — an occupancy grid, a decimated cloud, a
pose — not raw sensor streams.

## Phasing

**Phase 1 — Instrument.** Zenoh bridge to a laptop, confirm every topic, record
bags. Build the `1102` odometry bridge and validate it by walking a known
rectangle and measuring closure error. *Exit criterion: `/odom` and `/tf` exist and
are sane.*

**Phase 2 — Map.** FAST-LIO2 on front LiDAR + LiDAR IMU. Tune against recorded
bags. Produce a PCD and a 2-D grid of a real space. *Exit criterion: a
loop-closed map of a building floor with drift you can quantify.*

**Phase 3 — Localise.** Relocalise against a saved map on startup. This is what
makes missions repeatable. *Exit criterion: the robot recovers its pose in a known
map from an arbitrary starting point.*

**Phase 4 — Navigate.** Planner and controller emitting `Move()` at 50 Hz, with
ultrasonics and the rear LiDAR as a reactive safety layer. *Exit criterion: the
robot drives to a clicked point without hitting anything.*

**Phase 5 — Missions.** See [doc 05](05-mission-system.md).

Resist the temptation to start at phase 4. Each phase's exit criterion is
independently demonstrable, and each one is a thing that can be wrong in ways that
are invisible until the next phase makes them expensive.

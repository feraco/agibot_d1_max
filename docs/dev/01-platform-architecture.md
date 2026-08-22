# 01 — Platform Architecture

## The two-computer split

The D1 Max carries two compute boards with entirely separate roles and entirely
separate external interfaces. Almost every design decision downstream follows
from this split.

```
                     ┌────────────────────────────────────────┐
                     │              D1 Max                    │
                     │                                        │
  UDP  :8082  ◀──────┼─ RK3588 ──────────────┐                │
  WS   :8081  ◀──────┼─  motion control      │                │
  RTSP :8554  ◀──────┼─  system monitoring   │ wired LAN      │
                     │  192.168.234.1 (wifi) │ 192.168.168/24 │
                     │  192.168.168.168      │                │
                     │                       │                │
  ROS2/Zenoh  ◀──────┼─ Orin NX  ────────────┘                │
  :7447              │  perception, mapping,                  │
                     │  localisation, navigation              │
                     │  192.168.168.100                       │
                     │  157 TOPS, 16 GB                       │
                     └────────────────────────────────────────┘
```

`docs/source/1.6D1_Max硬件架构图.md` is explicit about the division of labour:

> RK3588 算力板负责运动控制和系统监控等功能，**勿在此板开发应用程序**
> ("do not develop applications on this board")
>
> Orin NX 算力板负责建图、定位、导航等业务功能，**用户可在此算力板开发轻量化的应用程序**
> ("users may develop lightweight applications on this board")

It also warns that flashing or an OTA upgrade will wipe whatever you put on the
Orin NX, so anything you deploy there needs to be reproducible from source and
backed up.

### Consequence

Motion commands and perception data arrive over two unrelated transports that
share no clock discipline, no coordinate frame convention, and no message
format. **Fusing them is your job.** This is the central integration task of the
whole project.

## Network map

| Endpoint | Address | Notes |
|---|---|---|
| RK3588 (Wi-Fi AP) | `192.168.234.1` | AP SSID `XG2WIFI_xxxxxx`, password `12345678` |
| RK3588 (wired) | `192.168.168.168` | ssh `robot`, password `bot` |
| Orin NX (wired) | `192.168.168.100` | ssh `robot`, password `1` |
| Control protocol (UDP) | `:8082` | **default transport since SDK v0.1.1** |
| Control protocol (WebSocket) | `:8081` | previous default |
| RTSP front camera | `rtsp://<rk3588>:8554/front` | H.264 |
| RTSP rear camera | `rtsp://<rk3588>:8554/back` | H.264 |
| Zenoh router (ROS 2) | `192.168.168.100:7447` | `ROS_DOMAIN_ID=24` |
| Front LiDAR | `192.168.1.102` | on its own subnet behind the Orin |
| Rear LiDAR | `192.168.2.102` | on its own subnet behind the Orin |

Your PC must take `192.168.168.x` where `x ∉ {100, 168, 255}`.

**Reaching the wired subnet while on Wi-Fi** (from `docs/source/5.7`) — this
matters a lot, because it is what lets a single client talk to *both* boards over
one Wi-Fi link:

```bash
# Linux
sudo ip route add 192.168.168.0/24 via 192.168.234.1
# Windows
route -p ADD 192.168.168.0 MASK 255.255.255.0 192.168.234.1
```

After this, `192.168.168.168` (control) and `192.168.168.100` (perception) are
both reachable from a laptop or phone associated to the robot's AP.

> **Note on ports.** The repository is inconsistent: the top-level `README.md`
> shows `./data ${ip} 8082`, `docs/source/2.7` shows `8081`, and the two copies
> of `example/control.cpp` print different examples. The protocol PDF is the
> authority: **UDP is 8082, WebSocket is 8081**, and the SDK's default transport
> became UDP in v0.1.1. Match the port to the transport, not to whichever example
> you copied.

## What the C++ SDK actually is

`SDKClient` (`include/robot_sdk/sdk_client.hpp`) is a pimpl-wrapped Boost.Beast /
Boost.Asio client that serialises a fixed set of commands into the JSON protocol
and dispatches inbound frames to two virtual-function callback interfaces. That
is the whole of it. Inspecting `librobot_sdk.so` confirms the internals:
`robot_sdk::WebSocketClient`, `robot_sdk::JsonCodec`, `nlohmann::json`,
`boost::beast::websocket`.

### Surface area

**Motion & posture** — `StandUp`, `LieDown`, `Crawl`, `Climb`, `Gait`, `Slim`,
`DSB` (mouse-barrier/kerb posture), `Locked`, `ReverseHeadTail`, `Move`, `Turn`,
`ControlHead`, `HighLowStance`, `SetMode`, `SetSpeed`.

**Safety** — `SoftEmergencyStop`.

**Lights** — `FrontLight`, `BackLight`, `AutoModeLight`.

**Telemetry configuration** — `SetImuConfig(freq)`, `SetLuxConfig`,
`SetMcConfig`, `SetSpeedReportConfig(on, freq)`, `SetJointStateConfig`.

**Control ownership** — `TakeControl`, `ReleaseControl`.

**Media** — `UpdateCameraBitrate`.

### Surface area it does *not* have

No mapping. No localisation. No path planning. No waypoints. No goal poses. No
docking/recharge call. No point-cloud access. No camera frame access (video is
RTSP only, out of band).

The optional SLAM/navigation and autonomous-recharge modules referenced in
`docs/source/2.2` and `5.2` are separate products. Notably, `docs/source/5.2`
runs the recharge demo as:

```
./control 192.168.168.168 8081 192.168.168.100 10010
```

— two endpoints, control board *and* Orin NX. That two-address signature is a
strong hint at the intended architecture for anything autonomous: **a process
that speaks the control protocol to the RK3588 and a service protocol to the Orin
NX simultaneously.** That process is what you are going to build. The demo
matching that invocation is not in this repository; the `control.cpp` here is a
keyboard teleop taking a single endpoint.

## Semantics you must respect

These are the behaviours that will bite you if you treat the SDK as a
request/response API.

### Move commands expire after one second

From the API reference: *"The latest Move command will last for 1 second."*
The protocol document is more specific about the intended cadence — a remote-control
client sends message `1003` at **50 Hz while the stick is off-centre and 5 Hz when
it is centred**.

So `Move()` is not "go 2 metres". It is "hold this velocity, and I promise to
tell you again shortly". Any controller you write needs a fixed-rate transmit
loop, not event-driven sends. If your loop stalls for over a second the robot
decelerates on its own — which is a useful safety property, and the reason you
should never work around it.

### Heartbeat is mandatory

Message `1001` at 5 Hz. Losing it raises `RobotRemoteKeepAliveFailure` and the
robot enters protection: it stops and lowers itself to the ground. The same
protection triggers on battery below 10%, joint faults, and IMU communication
loss.

### Speed level rescales `Move`

`Move` arguments are normalised `[-1.0, 1.0]` **percentages**, not velocities.
The mapping depends on `SetSpeed`:

| Level | `forward_back` ±1.0 | `left_right` ±1.0 | `yaw` ±1.0 |
|---|---|---|---|
| Low (1) | 1.0 m/s | 0.5 m/s | 1.5 rad/s |
| Medium (2) | 2.0 m/s | 0.5 m/s | 1.5 rad/s |
| High (3) | 3.0 m/s | 0.5 m/s | 1.5 rad/s |

At medium and high levels, lateral and yaw authority is progressively cut as
forward speed rises: above 1.0 m/s lateral is clamped to 0 and yaw drops to
1.0 rad/s; above 2.0 m/s yaw drops to 0.5 rad/s.

**This is a nonlinear, state-dependent actuator model.** A closed-loop waypoint
controller must either pin the speed level for the duration of a mission, or model
the change. Pin it. The spec sheet quotes a 6 m/s maximum for the platform, but
the SDK's `Move` path tops out at 3 m/s.

### Control ownership is preemptible and you always lose

From `docs/source/5.1` and `sdk_control_ownership_en.md`:

- The App **may** preempt the SDK at any moment.
- The SDK may **never** preempt the App.
- If the App connected first, the SDK cannot control at all — though it can still
  read state and trigger an emergency stop.

`IDataCallback::OnControlLost()` can fire mid-mission with no warning. Treat it as
a hard abort, not a warning: cancel the mission, stop the transmit loop, and
surface it loudly in the UI. `OnControlAvailable()` signals you may re-acquire —
but re-acquisition should require an operator action, never be automatic.

### State machine ordering

`docs/source/2.8` warns that issuing commands out of order can cause the robot to
fall or become unresponsive. Two rules worth encoding directly into your client
as a guard:

- Standing up auto-transitions into General / In-Place / Stair state depending on
  the current mode; you do not choose the destination state directly.
- `Locked` is exited by commanding any other state.

Build a state-machine guard that rejects illegal transitions client-side. A 41 kg
machine is not a good place to discover an ordering bug.

## Coordinate frames and extrinsics

`docs/source/2.10` gives sensor positions relative to the body BASE origin, in
millimetres, which is exactly what a SLAM stack needs for extrinsic calibration:

| Sensor | x | y | z |
|---|---|---|---|
| IMU ICM42688 (motion control) | 0 | 0 | 36.2 |
| IMU LUA300C (automotive grade) | 0 | 0 | 56.9 |
| Front LiDAR (Airy) | 404.3 | 0 | −37.7 |
| Front camera IMX415 | 412.3 | 0 | 37.8 |
| Rear LiDAR (Airy) | −404.3 | 0 | −37.7 |
| Rear camera IMX415 | −412.3 | 0 | 37.8 |
| Front-right ultrasonic | 179.2 | −100.2 | 50 |
| Front-left ultrasonic | 179.2 | 100.2 | 50 |

`urdf/max_description/urdf/max.urdf` carries the same links (`BASE_LINK`,
`IMU_ICM42688_LINK`, `IMU_LUA300C_LINK`, the `*_AIRY_LINK` and `*_IMX415_LINK`
frames, 28 joints) with meshes. It is your `robot_state_publisher` input and the
source of truth for the TF tree — publish from the URDF rather than hardcoding
the table above, so the two cannot drift apart.

Joint naming (`docs/source/2.9`): `{f,b}{l,r}{1..4}_{hip_roll,hip_pitch,knee_pitch,foot}`,
e.g. `fl2_hip_pitch`. These are the keys in `RobotState::joint_temps` and in
`JointStateData::names`.

## Platform limits worth designing around

From `docs/source/1.3`: 41 kg, IP67, 30 kg payload, 5 h unloaded endurance,
25 cm continuous stair height, 80 cm maximum obstacle, 45° maximum slope, 50 cm
minimum passage width. Two hot-swappable 504.9 Wh batteries.

The 50 cm minimum passage width and 25 cm step height are the numbers your path
planner needs. The dual-battery reporting in `BatteryData` (per-pack voltage,
current, temperature, presence, charge status) is granular enough to drive a
real return-to-charge policy rather than a single percentage threshold.

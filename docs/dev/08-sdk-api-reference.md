# 08 — SDK API Reference

Complete surface of `robot_sdk` (RobotSDK 0.1.1), assembled from the shipped
headers plus `docs/source/3.3`, `3.4`, `3.1`, `3.2` — the parameter *values*
(mode ids, speed scaling, stance codes) exist only in the docs, not the headers.

**36 public methods on `SDKClient`, 9 data callbacks, 23 control callbacks.**

Header: `include/robot_sdk/sdk_client.hpp` · namespace `robot_sdk`

---

## Calling convention — applies to every control method

Every control method has the same tail: `(..., int timeout_ms = 0, WriteHandler handler = {})`.

| `timeout_ms` | Mode | Behaviour |
|---|---|---|
| `0` | **async** | returns immediately; result arrives via `IControlCallback` |
| `> 0` | **sync** | blocks until acked or timeout |

`block` is different and applies **only** to `Connect` / `Disconnect`.

All methods return `std::error_code`. Success is `ec.value() == 0`.

| Code | Value | Meaning |
|---|---|---|
| `success` | 0 | ok |
| `invalid_argument` | 22 | bad parameter |
| `already_connected` | 106 | |
| `not_connected` | 107 | |
| `timed_out` | 110 | |
| `connection_refused` | 111 | |
| `connection_already_in_progress` | 114 | |
| `operation_in_progress` | 115 | |
| `operation_canceled` | 125 | |
| `robot_sdk::errc::ShakeHandFailed` | **10000** | no handshake reply |
| `robot_sdk::errc::ProtocolMismatch` | **10001** | robot firmware too old |
| `robot_sdk::errc::ControlledDenial` | **10002** | another client holds control |

---

## 1. Construction & connection — 6 methods

| Method | What it does |
|---|---|
| `SDKClient(ErrorHandler, ConnectionConfig, TransportProtocol)` | Construct. Defaults: empty error callback, default config, **`TransportProtocol::Udp`**. |
| `~SDKClient()` | Disconnects and frees resources. |
| `Connect(ip, port, block=false, handler)` | Connect to the robot. **UDP → port `8082`; WebSocket → port `8081`.** Fails with `ControlledDenial` if the handset App holds the session. |
| `Disconnect(block=false, handler)` | Close the session. |
| `IsConnected() const` | → `bool`. |
| `GetConnectionState() const` | → `ConnectionState`: `DISCONNECTING 0`, `DISCONNECTED 1`, `CONNECTING 2`, `HANDSHAKING 3`, `CONNECTED 4`, `RECONNECTING 5`. |

`ConnectionConfig`: `connect_timeout_ms` (default 5000, min 500), `auto_reconnect`
(default **false**), `reconnect_interval_ms` (default 1000, min 500).

Copy construction and assignment are **deleted** — `SDKClient` is non-copyable.

## 2. Callback registration — 2 methods

| Method | What it does |
|---|---|
| `SetControlCallback(shared_ptr<IControlCallback>)` | Register command acknowledgements. |
| `SetDataCallback(shared_ptr<IDataCallback>)` | Register telemetry. |

> Callbacks must be **lightweight** — copy the data out to your own thread.
> Blocking inside one stalls the SDK's receive path.

## 3. Safety — 1 method

| Method | What it does |
|---|---|
| `SoftEmergencyStop(bool on, …)` | `true` = engage. **While engaged the robot ignores all motion commands and holds speed at 0.** `false` = release. |

## 4. Posture & action — 9 methods

All parameterless apart from the standard tail.

| Method | What it does |
|---|---|
| `StandUp()` | Stand. On completion **auto-switches to General or In-Place** depending on the current mode. |
| `LieDown()` | Lie down. |
| `Crawl()` | Prone / crawl posture. |
| `Climb()` | Climb a high platform. |
| `Gait()` | Enter gait (walking) state. |
| `Slim()` | "Body compress" — narrows the stance to fit through gaps. |
| `DSB()` | Kick-plate / threshold-crossing posture (过挡鼠板姿态). |
| `ReverseHeadTail()` | Swap which end is treated as the head. |
| `Locked()` | Lock every joint at its current position. **Any posture command (stand/crawl/lie) auto-unlocks.** |

## 5. Mode & speed — 2 methods

| Method | Values | What it does |
|---|---|---|
| `SetMode(int mode, …)` | `1` General · `2` In-Place · `3` Stair | Operating mode. Default General. **Gates which motion methods work — see §6.** |
| `SetSpeed(int speed_level, …)` | `1` Low · `2` Medium · `3` High | Rescales `Move`. Default Low. |

### Speed scaling — the safety-critical table

`Move` takes normalised ±1.0. What that *means* depends on the level, and at
Medium/High **lateral translation is cut off entirely above 1 m/s forward**:

| Level | forward_back | left_right | yaw |
|---|---|---|---|
| **1 Low** | ±1.0 m/s | ±0.5 m/s | ±1.5 rad/s |
| **2 Medium** | ±2.0 m/s | ±0.5 m/s if \|fwd\| < 1.0 m/s, else **0** | ±1.5 rad/s if \|fwd\| < 1.0, else ±1.0 |
| **3 High** | ±3.0 m/s | ±0.5 m/s if \|fwd\| < 1.0 m/s, else **0** | ±1.5 if \|fwd\| < 1.0 · ±1.0 if < 2.0 · **±0.5 above 2.0** |

Any UI must show the active level next to the stick — identical deflection is
1 m/s or 3 m/s depending on a setting the operator may have made minutes ago.

## 6. Motion & pose — 4 methods

**Mode-gated. Calling one in the wrong mode does nothing.**

| Method | Mode | What it does |
|---|---|---|
| `Move(float left_right, float forward_back, float yaw, …)` | **General only** | Velocity command, each ±1.0, scaled per §5. `left_right` **+ = translate left**; `forward_back` **+ = forward**; `yaw` **+ = rotate left**. **The command persists 1 second, then expires — it is a dead-man switch, so resend at ~50 Hz.** |
| `Turn(int direction, …)` | **In-Place only** | 翻滚 — a **roll**, not a yaw. `0` recover · `1` roll left · `2` roll right. |
| `ControlHead(float left_right, float up_down, …)` | **In-Place only** | Head look. ±1.0, rad/s. `left_right` + = look left; `up_down` + = look up. |
| `HighLowStance(int stance, …)` | **In-Place only** | `0` recover · `1` high stance · `2` low stance. |

> Note the argument order on `Move`: **left_right comes first**, not forward.
> This is the opposite of the usual `(vx, vy, wz)` convention and is an easy
> way to drive a 41 kg robot sideways into a wall.

## 7. Lights — 3 methods

| Method | What it does |
|---|---|
| `FrontLight(bool on, …)` | Front fill light. **Setting it turns auto mode off.** |
| `BackLight(bool on, …)` | Rear fill light. Same auto-mode side effect. |
| `AutoModeLight(bool on, …)` | Light follows the ambient lux sensor. |

## 8. Telemetry configuration — 5 methods

**Every stream is off by default.** Nothing arrives until you enable it.

| Method | Parameter | Resulting rate |
|---|---|---|
| `SetImuConfig(int freq, …)` | `[0, 100]` Hz; `0` = off | as requested → `OnImuData` |
| `SetLuxConfig(bool on, …)` | on/off | fixed **1 Hz** → `OnLuxData` |
| `SetMcConfig(bool on, …)` | on/off | fixed **50 Hz** → `OnMcData` |
| `SetSpeedReportConfig(bool on, uint32_t frequency, …)` | `[1, 50]` Hz | as requested → `OnSpeedData` |
| `SetJointStateConfig(bool on, …)` | on/off | → `OnJointStateData` |

`SetMcConfig` is the important one — `MotionData` at 50 Hz is **the robot's only
pose source**. There is no `/odom` topic; see `tools/d1max_odom_bridge.py`.

`SetJointStateConfig` is present in the header but **absent from the 3.3 docs** —
undocumented, treat its behaviour as unverified.

## 9. Control ownership — 2 methods

| Method | What it does |
|---|---|
| `TakeControl(…)` | Request control. **Fails if the handset App holds it — the SDK cannot preempt the App.** |
| `ReleaseControl(…)` | Give control back. |

The asymmetry matters: **App can take control from SDK at any time; SDK can
never take it from App.** Losing it fires `OnControlLost` while your last `Move`
is still within its 1-second window — treat the callback as a hard stop.

## 10. Camera — 1 method

| Method | What it does |
|---|---|
| `UpdateCameraBitrate(CameraBitrateCmd cmd, …)` | Set stream bitrate. `camera_name` is `"camera_front"` or `"camera_back"`; `camera_bps` range **50 000 – 100 000 000**. |

## 11. Version — 3 methods

`Version()`, `ProtocolVersion()`, `SystemVersion()` — all `const std::string&`.
Check `ProtocolVersion()` against the robot to pre-empt error 10001.

---

# Callbacks

## `IDataCallback` — 9 methods, telemetry

| Method | Payload | When |
|---|---|---|
| `OnImuData(const ImuData&)` | `acc_{x,y,z}`, `gyro_{x,y,z}`, `quat_{x,y,z,w}` | after `SetImuConfig` |
| `OnLuxData(const LuxData&)` | `lux` | 1 Hz after `SetLuxConfig` |
| `OnMcData(const MotionData&)` | `quat[4]` **[w,x,y,z]**, `v_world[3]`, `position[3]`, `omega_world[3]`, `v_body[3]`, `omega_body[3]`, `time_stamp` (**ns**) | 50 Hz after `SetMcConfig` |
| `OnSpeedData(const SpeedData&)` | `x`, `y`, `yaw` | after `SetSpeedReportConfig` |
| `OnJointStateData(const JointStateData&)` | `names[]`, `positions[]`, `velocities[]`, `efforts[]` | after `SetJointStateConfig` |
| `OnRobotStateData(const RobotState&)` | see below | **1 Hz, always on** |
| `OnFaultData(const FaultDatas&)` | `vector<FaultData>` | on fault |
| `OnControlLost(const ControlLostInfo&)` | empty struct | you were preempted |
| `OnControlAvailable(const ControlAvailableInfo&)` | empty struct | control freed up |

`OnRobotStateData` is the only stream that needs no configuration.

### `RobotState`

`head_angle`, `front_fill_light`, `back_fill_light`, `auto_mode_light`,
`speed_level`, `software_emergency_status`, `hardware_emergency_status`
(**two independent e-stops**), `head_direction`, `motion_status`, `battery`,
`speed`, `mile_data`, `joint_temps` (name → °C), `sport_mode`, `control_source`.

`BatteryData` covers **two packs**: `power{1,2}`, `present{1,2}`, `voltage{1,2}`,
`temperature{1,2}`, `current{1,2}`, `power_supply_status{1,2}`.

### State enums

- `MotionStatus`: `UNKNOWN 0`, `STAND_UP`, `LIE_DOWN`, `CRAWL`, `LOCKED`, `GENERAL`, `IN_PLACE`, `STAIR`, `CLIMB`, `SLIM`, `GAIT`
- `SportMode`: `UNKNOWN 0`, `GENERAL`, `IN_PLACE`, `STAIR`
- `SpeedLevel`: `UNKNOWN 0`, `SLOW`, `MEDIUM`, `HIGH`
- `CtrlSource`: `UNKNOWN 0`, `APP 1`, `SDK 2`, `OTHER 3`
- `EmergencyStatus`: `UNKNOWN 0`, `RECOVER`, `STOP`
- `HeadDirection`: `UNKNOWN 0`, `HEAD`, `TAIL`
- `PowerSupplyStatus`: `UNKNOWN 0`, `CHARGING 1`, `DISCHARGING 2`, `FULL 4`
- `FillLightStatus`: `UNKNOWN 0`, `ON`, `OFF`

### `FaultCode` / `FaultLevel`

`FaultLevel`: `Unknown 0`, `FatalError`, `Error`, `Warn`.

| Code | Value | Meaning |
|---|---|---|
| `Unknown` | 0 | |
| `ActuatorDisabled` | **10** | actuator disabled |
| `ActuatorEncoderError` | 11 | encoder fault |
| `ActuatorOffline` | 12 | actuator dropped off the bus |
| `ActuatorOverVoltage` | 13 | |
| `ActuatorOverheat` | 14 | |
| `ActuatorTempWarn` | 15 | |
| `ActuatorTimeout` | 16 | control timeout |
| `ActuatorUndervolt` | 17 | |
| `PowerControlOverTemp` | 18 | battery overheat |
| `PowerControlPowerEmpty` | 19 | **< 10 %** |
| `PowerControlPowerLow` | 20 | 10–20 % |
| `PowerControlOffline` | 21 | power board MCU unreachable |
| `CANBroken` | 22 | CAN bus error |
| `RobotRemoteKeepAliveFailure` | 23 | remote control disconnected |
| `SystemClockSanityError` | 24 | **system time jumped** |
| `SystemRobotStatusError` | 25 | robot status abnormal |
| `IMUConnectError` | 26 | |
| `IMUDataNotUpdated` | 27 | IMU stale |

`SystemClockSanityError` is worth surfacing loudly — a clock jump silently
corrupts any mapping run in progress.

## `IControlCallback` — 23 methods, command acknowledgements

Used in async mode to confirm the robot **received** a command.

| Group | Callbacks |
|---|---|
| Safety | `OnSoftEmergencyStop(bool)` |
| Posture | `OnStandUp()`, `OnLieDown()`, `OnCrawl()`, `OnClimb()`, `OnSlim()`, `OnGait()`, `OnDSB()`, `OnReverseHeadTail()`, `OnLocked()` |
| Mode | `OnMode(int)`, `OnSpeed(int)` |
| Lights | `OnFrontLight(bool)`, `OnBackLight(bool)`, `OnAutoModeLight(bool)` |
| Config | `OnLuxConfig(bool)`, `OnImuConfig(int)`, `OnMcConfig(bool)`, `OnSpeedReportConfig(bool, uint32_t)`, `OnJointStateConfig(bool)` |
| Ownership | `OnTakeControlAck(const TakeControlAck&)`, `OnReleaseControlAck(const ReleaseControlAck&)` |
| Camera | `OnUpdateCameraBitrateAck(const CameraBitrateAck&)` |

`TakeControlAck` / `ReleaseControlAck`: `uint32_t error_code` (0 = ok) plus a
`std::string reason`.

> **There is no ack for `Move`, `Turn`, `ControlHead`, or `HighLowStance`.**
> The four motion methods are fire-and-forget; confirm them by watching
> `OnMcData` / `OnRobotStateData`, not by waiting for a callback.

---

## What the C++ SDK does *not* expose

The wire protocol carries commands with no `SDKClient` method behind them.
`tools/d1max_proto.py` speaks these directly:

| Wire command | Note |
|---|---|
| `mode/navigation` | `SetMode` accepts only 1–3 |
| `mode/follow`, `mode/track` | not on ZSM-1 / ZSM-1F |
| `knee_mode/same_direction`, `knee_mode/medial_facing` | not on point-foot variants |
| `action/new1new` | "wiggle" |
| `emergency/recover` | `SoftEmergencyStop(false)` is the SDK path |

This is one reason the tooling in `tools/` reimplements the protocol rather
than binding to `librobot_sdk.so`. The other is portability — the shared
object is glibc + GNU libstdc++ with **zero unmangled exports**, so it cannot
be loaded from Android (bionic) or iOS, and there is no C ABI to bind from
Python, Rust, or Go.

---

## Obstacle avoidance and SLAM — neither is in the SDK

Asked often enough to be worth stating flatly: **RobotSDK 0.1.1 has no obstacle
avoidance and no mapping/navigation API.** Not "undocumented" — absent.

- `SDKClient` has **no** nav, avoidance, map, goal or path method among its 36.
- `IDataCallback` has **no** LiDAR, ultrasonic or map callback among its 9.
- `避障` (obstacle avoidance) appears in exactly **one** file across the whole
  documentation set: `2.2 SDK软件服务接口列表`, the service *roadmap* table
  ("超声波雷达接口 / 实现自主避障功能 / v0.0.5").
- `SLAM` / `建图` appears in **two**: that same roadmap table — marked
  **选配, optional/extra-cost** — and `1.6`, which only says the Orin NX
  "负责建图、定位、导航等业务功能" (handles mapping, localisation, navigation).
- No `/map`, `/scan`, `/cmd_vel`, `/nav*`, costmap or goal topic is documented
  anywhere.

**Nothing stops this robot from walking into a wall.** Any autonomy has to be
built on the raw sensor streams below.

### What you actually get — ROS 2 on the Orin NX, not the SDK

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/front_lidar` | `sensor_msgs/PointCloud2` | 10 Hz | QoS **best_effort**. Sensor IP `192.168.1.102` |
| `/rear_lidar` | `sensor_msgs/PointCloud2` | 10 Hz | QoS **best_effort**. Sensor IP `192.168.2.102` |
| `/uss_driver/uss_left/range` | `sensor_msgs/Range` | 10 Hz | 0–4 m |
| `/uss_driver/uss_right/range` | `sensor_msgs/Range` | 10 Hz | 0–4 m |

```bash
ssh robot@192.168.168.100          # password: 1
ros2 topic echo /front_lidar --once
ros2 topic echo /uss_driver/uss_left/range --once
```

> **Ultrasonic trap.** `docs/source/4.3` documents `field_of_view`, `min_range`
> and `max_range` as **fixed at 0** — the driver does not populate them. You get
> a bare distance scalar with no cone geometry, so the FOV has to be hardcoded
> from the datasheet. Only two sensors, left and right: there is **no forward
> ultrasonic**, so they cover flanks, not travel direction.

Two LiDARs at 10 Hz is a workable avoidance input, but you write the layer.

### The safety systems that do exist (`docs/source/2.3`)

Reactive stops, **none obstacle-triggered**:

| Trigger | Effect | Recovery |
|---|---|---|
| Hard e-stop button | lowers slowly to the ground, red light | twist the button out; auto-recovers if no other fault |
| Soft e-stop (SDK or RC) | stops immediately, ignores all control | release the soft stop |
| RC/network disconnect (incl. congestion) | auto-stop, slow descent | reconnect the App |
| Battery present and **< 10 %** | auto-stop, slow descent | charge |
| Joint fault | auto-stop, slow descent | clear the fault |
| IMU comms loss | auto-stop, slow descent | — |

From SDK v0.0.6, to trigger a soft e-stop from the handset *while the SDK holds
control*, you must open the App after `TakeControl` — then the RC's red button
works at any time. Worth knowing before a field test.

### The one autonomous behaviour that ships — and it is not navigation

`docs/source/5.2`, autonomous recharge: **optional**, needs RK3588 ≥ 0.2.4 and
Orin NX ≥ 0.4.5, plus the dock. It is explicitly **无图回充 — *mapless*
recharge**: place the robot facing the dock, ~1.5 m away, with the dock's QR
code fully in the camera's view. That is visual servoing onto a fiducial, not
path planning.

```bash
./control 192.168.168.168 8081 192.168.168.100 10010
```

Note the demo talks to **two** endpoints — the RK3588 on `8081` (WebSocket, not
the usual UDP 8082) and a separate Orin NX service on **`10010`**, which is
otherwise undocumented and may be where the optional SLAM package lives.

### Consequence for this repo

This gap is the reason `tools/` exists. `d1max_odom_bridge.py` supplies the
missing motion prior, `d1max_map.py` drives FAST-LIO2 on the Orin, and
`d1max_slam.py` projects the cloud to a nav2 grid — all of it building the layer
the SDK does not provide. See `docs/dev/03` and `docs/dev/07`.

---

## Minimal working sequence

```cpp
#include "robot_sdk/sdk_client.hpp"
using namespace robot_sdk;

auto data = std::make_shared<MyDataCallback>();      // : IDataCallback
SDKClient sdk([](const std::error_code& e){ /* transport errors */ });
sdk.SetDataCallback(data);

if (auto ec = sdk.Connect("192.168.234.1", "8082", /*block=*/true)) return 1;

sdk.TakeControl(1000);          // fails 10002 if the App holds control
sdk.SetMcConfig(true, 1000);    // 50 Hz pose — off until you ask
sdk.SetSpeed(1, 1000);          // Low
sdk.SetMode(1, 1000);           // General — required before Move works
sdk.StandUp(3000);

// Move expires after 1 s. Resend at ~50 Hz or the robot stops.
while (driving) {
  sdk.Move(/*left_right=*/0.0f, /*forward_back=*/0.3f, /*yaw=*/0.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

sdk.Move(0, 0, 0);
sdk.LieDown(3000);
sdk.ReleaseControl(1000);
sdk.Disconnect(true);
```

Order matters: `TakeControl` → `SetMode(1)` → `StandUp` → `Move`. Calling
`Move` in In-Place mode, or before standing, silently does nothing.

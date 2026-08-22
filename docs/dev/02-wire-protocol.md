# 02 — Wire Protocol Reference

Source: `ubuntu22.04(x86_64_arm64)/RobotSDK-0.1.1/docs/protocol/Protocol-1.2.0.pdf`
("中狗APP&SDK通信协议-1.2.0-对外开放版本" — externally-released version),
cross-checked against the SDK headers and against strings in `librobot_sdk.so`.

This is the interface the shipped `librobot_sdk.so` speaks. **It is documented and
open**, which means you can implement a first-class client in Kotlin, Swift, Rust,
Dart, Python or anything else without linking a single line of vendor code. That
fact is the foundation of the mobile plan in [doc 04](04-clients-linux-android-ios.md).

## Transport

The robot body is the **server**; your application is the client.

| Transport | Endpoint |
|---|---|
| UDP | `192.168.234.1:8082` |
| WebSocket | `192.168.234.1:8081` |

Substitute `192.168.168.168` when connected over the wired interface. UDP became
the SDK default in v0.1.1; the WebSocket path remains supported.

## Framing

Every APDU is a fixed 16-byte header followed by one ASDU (the JSON payload).

```
 0        1        2        3        4        5        6        7
┌────────┬────────┬────────┬────────┬─────────────────┬─────────────────┐
│  0x5A  │  0x53  │  0x4B  │  0x4A  │   length  (u16) │   msg_id  (u16) │
│   'Z'  │   'S'  │   'K'  │   'J'  │   little-endian │   little-endian │
└────────┴────────┴────────┴────────┴─────────────────┴─────────────────┘
 8 ─────────────────────────────────────────────────────────────────── 15
┌───────────────────────────────────────────────────────────────────────┐
│                    reserved — 8 bytes, zero-filled                    │
└───────────────────────────────────────────────────────────────────────┘
                                  ↓
                    ASDU: UTF-8 JSON, `length` bytes
```

- **Sync word** `5A 53 4B 4A` = ASCII `ZSKJ`.
- **length** — ASDU byte count, little-endian, max 65535.
- **msg_id** — per-frame identifier used to correlate a request with its
  response. The requester owns the value; the responder echoes it. Increment from
  0, wrap at 65535. Little-endian.

A tested, dependency-free encoder/decoder is at
[`tools/d1max_proto.py`](../../tools/d1max_proto.py).

## ASDU envelope

```json
{
  "head": {
    "type": 1002,
    "time": 1757310525610,
    "src":  3
  },
  "data": { }
}
```

| Field | Type | Meaning |
|---|---|---|
| `type` | uint32 | Message type (table below) |
| `time` | uint64 | Send timestamp, **milliseconds** |
| `src` | uint32 | `1` body · `2` App · `3` SDK · `4` EXTERNAL (direct protocol client) |

Pick `src` deliberately. `3` (SDK) inherits SDK ownership semantics — the App can
preempt you. `4` (EXTERNAL) is the identity for a direct protocol client. Use `3`
if you want your client to behave exactly like the shipped SDK, which is normally
what you want, because the ownership rules are the safety model.

> Several JSON samples in the PDF contain full-width commas (`，`) and trailing
> commas — transcription artefacts of the document, not the protocol. Emit strict
> JSON.

## Message types

| Type | Name | Direction | Notes |
|---|---|---|---|
| 1000 | Handshake | ext → body, reply body → ext | **Mandatory first message** |
| 1001 | Heartbeat | ext → body, reply body → ext | 5 Hz, mandatory |
| 1002 | Command | ext → body, ack body → ext | Discrete commands |
| 1003 | Teleop | ext → body | Continuous velocity, 50 Hz / 5 Hz |
| 1004 | Body state | body → ext | 1 Hz |
| 1005 | Fault state | body → ext | On fault |
| 1008 | Sensor report config | ext → body, ack body → ext | SDK/EXTERNAL only |
| 1013 | Take control | ext → body, ack body → ext | **App-initiated preemption only** |
| 1015 | Release control | ext → body, ack body → ext | |
| 1016 | Control taken (notification) | body → ext | → `OnControlLost` |
| 1017 | Control released (notification) | body → ext | → `OnControlAvailable` |
| 1018 | Camera bitrate | ext → body, ack body → ext | |
| 1050 | Goodbye / disconnect | ext → body, ack body → ext | Lets the UDP server detect client teardown promptly |
| 1100 | IMU data | body → ext | 100 Hz fixed |
| 1101 | Light-level data | body → ext | 1 Hz fixed |
| 1102 | Motion-control data | body → ext | **50 Hz fixed — the odometry source** |

### 1000 — Handshake

Must be the first message after connecting; it retrieves the body's version
information and gates everything else.

Request `data`:

| Field | Type | Notes |
|---|---|---|
| `version` | string | Your software version |
| `protocol_version` | string | `"1.2.0"` — format `xx.xx.xx` |
| `device` | string | Device name |
| `platform` | string | e.g. `"ios"` |
| `package_name` | string | Application unique identifier |

Response `data`:

| Field | Type | Notes |
|---|---|---|
| `status_code` | uint32 | `0` success · `10` protocol version mismatch · `20` already controlled by another terminal |
| `sn` | string | Robot serial number |
| `ssid` | string | |
| `model` | string | |
| `version.system` | string | |
| `version.charging_pile` | string | |
| `device_type` | string | `ZSM-1` wheel-foot · `ZSM-1F` point-foot · `-Pro` LiDAR variants · `-Ultra` surround-view variants |

`status_code` maps onto the SDK's error codes: `10` → `ProtocolMismatch` (10001),
`20` → `ControlledDenial` (10002), and a missing/failed handshake →
`ShakeHandFailed` (10000).

**Read `device_type` and branch on it.** Several commands are unsupported on
point-foot and base variants — see the command table.

### 1001 — Heartbeat

5 Hz, no `data` payload required. The response echoes the request's `time` value,
which gives you a free round-trip-time measurement — use it to drive a link-quality
indicator, and to distinguish "robot stopped responding" from "Wi-Fi got slow".

### 1002 — Command

`data` is a single field, `cmd`, a string:

| Group | Command | String |
|---|---|---|
| Emergency | stop | `emergency/stop` |
| | recover | `emergency/recover` |
| Action | stand up | `action/stand_up` |
| | crawl | `action/crawl` |
| | lie down | `action/lie_down` |
| | lock | `action/locked` |
| | climb platform | `action/climb` |
| | kerb / mouse-barrier posture | `action/dsb` |
| | narrow-gap posture | `action/slim` |
| | gait walk posture | `action/gait_walk` |
| | wiggle | `action/new1new` |
| Head | reverse head/tail | `reverse_head_tail` |
| Mode | general | `mode/general` |
| | in place | `mode/in_place` |
| | **navigation** | `mode/navigation` |
| | stair | `mode/stair` |
| | follow | `mode/follow` ¹ |
| | track | `mode/track` ¹ |
| Speed | low | `speed/low` |
| | medium | `speed/medium` |
| | high | `speed/high` |
| Knee | same-direction | `knee_mode/same_direction` ² |
| | medial-facing | `knee_mode/medial_facing` ² |
| Fill light | auto on / off | `fill_light/light_auto_work_on` / `..._off` |
| | front on / off | `fill_light/front_light_on` / `fill_light/front_light_off` |
| | rear on / off | `fill_light/back_light_on` / `fill_light/back_light_off` |

¹ Unsupported on `ZSM-1` and `ZSM-1F`.
² Unsupported on all point-foot variants (`ZSM-1F`, `ZSM-1F-Pro`, `ZSM-1F-Ultra`).

The body echoes the same `cmd` string back as an acknowledgement, meaning "received"
— **not** "completed". Motion completion must be inferred from the `1004` state
stream. Design your command layer around that: an ack clears the retry timer, a
state change resolves the operation.

Three commands here are absent from the C++ `SDKClient` API: `mode/navigation`,
`mode/follow`, `mode/track`, plus the knee modes and `action/new1new`. A direct
protocol client can reach capability the shipped SDK does not expose. Whether
`mode/navigation` does anything useful without the optional navigation module is
untested — treat it as a lead to verify on hardware, not as a shortcut.

### 1003 — Teleop

The continuous-velocity channel behind `SDKClient::Move`, `Turn`, `ControlHead`
and `HighLowStance`.

| Field | Type | Range | Meaning |
|---|---|---|---|
| `lx` | float | [−1, 1] | Motion mode: translation. Positive forward, negative back. In-place: no effect |
| `ly` | float | [−1, 1] | Motion mode: positive strafes left, negative right. In-place: no effect |
| `rx` | float | [−1, 1] | Motion mode: yaw, positive left. In-place: head pan, positive left |
| `ry` | float | [−1, 1] | Motion mode: no effect. In-place: pitch, positive head-up |
| `body.turn` | string | | In-place: `left` / `right` / `none` — horizontal body turn |
| `body.high_low` | string | | In-place: `up` / `down` / `none` — stance height |

Cadence rules from the specification:

- Sticks off-centre → **50 Hz**.
- Sticks centred → **5 Hz**.
- When `body.turn` or `body.high_low` are anything other than `none`, send the
  stick values as zero.

> **Known documentation inconsistency.** The PDF's prose for `lx` says "本体左右方向的
> 移动" (left/right movement) while the same sentence says positive is forward and
> negative is backward. The C++ signature is `Move(left_right, forward_back, yaw)`,
> and `example/control.cpp` maps `w`/`s` (forward/back) to the *second* argument
> and `a`/`d` to the first. Reading the two together, the SDK's `forward_back`
> and `left_right` most plausibly correspond to `lx` and `ly` respectively, with
> the PDF's prose label for `lx` being an error. **Verify this on hardware before
> trusting it**, at low speed, in an open space, with a hand on the e-stop. Getting
> the axes swapped on a 41 kg robot is the kind of mistake that is cheap to test
> for and expensive to discover.

### 1004 — Body state, 1 Hz

Populates `RobotState`: temperature, `head_angle`, `head_direction`
(`"head"`/`"tail"`), knee mode (`"same"`/…), fill-light states, speed level,
software and hardware emergency-stop states, motion status, dual-battery data,
current velocity, cumulative mileage, per-joint temperatures, sport mode, and
control source. See `include/robot_sdk/sdk_type.hpp` for the full decoded shape
and the enumerations.

`control_source` (`CTRL_SOURCE_APP` / `SDK` / `OTHER`) is the field that tells you
whether you currently hold control. Poll it as a backstop to the `1016`/`1017`
notifications — a lost UDP datagram must not leave your client believing it is
still in charge.

### 1005 — Fault state

Array of faults, each with a code, level and message; see `FaultCode` and
`FaultLevel` in `sdk_type.hpp`. Levels are `FatalError`, `Error`, `Warn`. Codes
cover actuator faults (disabled, encoder, offline, over/under-voltage, overheat,
timeout), power faults (over-temperature, <10%, <20%, MCU offline), CAN errors,
remote keep-alive failure, system clock jumps, and IMU faults.

`RobotRemoteKeepAliveFailure` is the one you will hit during development: it means
your heartbeat lapsed.

### 1008 — Sensor report configuration

Nothing streams until you ask for it. This is the message behind `SetImuConfig`,
`SetLuxConfig`, `SetMcConfig`, `SetSpeedReportConfig` and `SetJointStateConfig`.

| `sensor` | Meaning | Resulting stream |
|---|---|---|
| 10 | IMU | `1100` @ 100 Hz |
| 20 | Light sensor | `1101` @ 1 Hz |
| 30 | Motion control | `1102` @ 50 Hz |
| 40 | Body speed | configurable 1–50 Hz |
| 50 | JointState | |

Plus `enable` (bool) and `freq` (uint32). **Only sensor 40 currently honours
`freq`**; the others run at fixed rates. The `SetImuConfig(freq)` signature accepts
`[0, 100]` with 0 disabling, so treat IMU frequency as on/off in practice.

### 1013 / 1015 / 1016 / 1017 — Control ownership

`1013` take, `1015` release; both acked with `error_code` (0 = OK) and a `reason`
string. `1016` and `1017` are unsolicited notifications that ownership was taken
from you or released, and map to `OnControlLost` / `OnControlAvailable`.

The specification states plainly for `1013`: **仅允许APP发起抢占** — only the App
may initiate preemption. Your client asks; it does not take.

### 1050 — Goodbye

Exists so a UDP server can promptly notice a client going away rather than waiting
for a timeout. **Send it on clean shutdown.** Skipping it leaves the robot believing
a stale client still holds a session, which is a genuinely annoying failure mode to
debug on the next connect. Wire it into your app-teardown path, and on mobile into
background/terminate lifecycle callbacks.

### 1100 / 1101 / 1102 — Telemetry

`1100` IMU @ 100 Hz — `a_x/a_y/a_z`, `g_x/g_y/g_z`, `q_w/q_x/q_y/q_z`.

`1101` light level @ 1 Hz — `lux`.

`1102` motion control @ 50 Hz — arrays `acc`, `gyro`, `omega_body`, and per
`MotionData` in `sdk_type.hpp`: `quat[4]` (w,x,y,z), `v_world[3]`, `position[3]`
(metres), `omega_world[3]`, `v_body[3]`, `omega_body[3]`, and `time_stamp` in
**nanoseconds**.

**`1102` is the most valuable message on this interface.** It is a 50 Hz
6-DoF state estimate with a high-resolution timestamp, and it is the only
odometry-like signal the platform offers. [Doc 03](03-slam-mapping-plan.md) is
built around it.

## Client state machine

```
   ┌──────────────┐
   │ DISCONNECTED │◀──────────────────────────┐
   └──────┬───────┘                           │
          │ open socket                       │ 1050 goodbye
          ▼                                   │ / timeout
   ┌──────────────┐                           │
   │  CONNECTING  │                           │
   └──────┬───────┘                           │
          │ send 1000 handshake               │
          ▼                                   │
   ┌──────────────┐  status_code ≠ 0          │
   │ HANDSHAKING  ├───────────────────────────┤
   └──────┬───────┘  10 mismatch / 20 denied  │
          │ status_code = 0                   │
          ▼                                   │
   ┌──────────────┐                           │
   │  CONNECTED   ├───────────────────────────┘
   └──────────────┘
      │        ▲
      │ 1001 heartbeat @ 5 Hz  (mandatory)
      │ 1003 teleop @ 50 Hz active / 5 Hz idle
      │ 1008 sensor enable (once, after connect)
      ▼
   ┌──────────────┐
   │ RECONNECTING │  auto_reconnect only; never on intentional disconnect
   └──────────────┘
```

Mirrors `ConnectionState` in `sdk_connection.hpp`. Reimplement it exactly — the
SDK's semantics (reconnect only on *unexpected* loss, minimum 500 ms interval and
timeout) are sensible and worth preserving.

## Implementation checklist

Everything below is required for a correct client, in rough order of how badly it
hurts to omit:

- [ ] Handshake before anything else; branch on `status_code` and `device_type`
- [ ] 5 Hz heartbeat on its own timer, independent of UI activity
- [ ] Fixed-rate teleop loop — 50 Hz active, 5 Hz idle; never event-driven
- [ ] Watchdog: zero the teleop vector if the UI/mission stops producing setpoints
- [ ] `msg_id` allocation and request/response correlation with timeouts
- [ ] `1008` enables re-sent after every reconnect (the robot does not remember)
- [ ] Handle `1016` as a hard mission abort
- [ ] Poll `control_source` in `1004` as a backstop for missed notifications
- [ ] Send `1050` on shutdown, and on mobile app-background
- [ ] Length-prefix reassembly on WebSocket; per-datagram parsing on UDP
- [ ] Reject frames without the `ZSKJ` sync word rather than attempting resync
- [ ] Clamp all teleop values to [−1, 1] before transmit — do not trust the caller

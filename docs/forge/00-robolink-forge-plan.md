# Robolink Forge — Port Plan

**Product:** Robolink Forge
**Vendor:** Robo Inc
**Target robot:** AgiBot D1 Max (quadruped)
**Reference app:** `szwedk/agi-dashboard` (built for X2 / Cadet)

---

## 0. Status of the reference read

**I could not read `szwedk/agi-dashboard`.** It is private and sits under a
different owner than this session's sources, so every route is closed:

| Attempt | Result |
|---|---|
| `add_repo szwedk/agi-dashboard` | `cross-tier adds are not supported in v1` |
| `git clone` through the proxy | prompts for credentials, no injection |
| `api.github.com/repos/szwedk/agi-dashboard` | `403` from the agent proxy |
| GitHub MCP `get_file_contents` | `Access denied: not configured for this session` |

`list_repos` confirms the repo exists, is private, and you have push rights —
the block is session scope, not permissions.

**To unblock, do one of these:**

1. **Mirror it under `feraco`** (fastest, keeps this session's context):
   ```bash
   git clone --mirror https://github.com/szwedk/agi-dashboard.git
   cd agi-dashboard.git
   # create an empty feraco/agi-dashboard on GitHub first
   git push --mirror https://github.com/feraco/agi-dashboard.git
   ```
   Then tell me — `feraco/*` is same-owner, so I can attach and read it
   immediately.

2. **Start a fresh session** seeded with `szwedk/agi-dashboard` as the initial
   source. Costs you this session's D1 Max context.

Everything in §1–§3 below is fixed by the D1 Max hardware and protocol and is
correct regardless of what the reference app turns out to look like. §4 is the
mapping table that gets filled in the moment I can read it.

### What I *was* able to inspect

- `feraco/x2` — URDF, meshes, RViz launch for the X2 Ultra. Robot description
  only, no app.
- `feraco/robo-teleop` — "Robostore Teleop", a FastAPI + WebSocket dashboard
  (`server.py`, 4472 lines, ~37 routes, single-file `static/index.html`).
  Not the reference, but it establishes the house pattern, and it is the same
  shape as the D1 Max console already in this repo.

---

## 1. The one thing that makes this not a find-and-replace

The brief was "make everything the same except the API calls." Two facts break
that, and both are worth deciding on before any code is written.

### 1.1 X2 is a humanoid. D1 Max is a quadruped.

They do not share a command vocabulary. A dashboard built for X2 / Cadet will
have controls with **no D1 Max equivalent**, and the D1 Max has controls with
**no X2 equivalent**:

| X2 / Cadet has | D1 Max equivalent |
|---|---|
| Arm / shoulder / elbow / wrist joint control | none — no arms |
| Hand / fist / gripper (`x2_hand`, `x2_fist`) | none |
| Waist yaw + pitch | none |
| Head pan/tilt | **`ControlHead(left_right, up_down)`** — In-Place mode only |
| Whole-body pose / retargeting | none |
| Bipedal balance & step planning | replaced by gait modes |

| D1 Max has | X2 equivalent |
|---|---|
| `action/crawl`, `action/climb`, `action/dsb`, `action/slim` | none |
| `mode/stair`, `mode/in_place`, `mode/follow`, `mode/track` | none |
| `knee_mode/same_direction`, `knee_mode/medial_facing` | none |
| Fill lights (front / back / auto) | none |
| `reverse_head_tail` | none |

**Recommendation:** keep the reference app's *shell* — layout, navigation,
panel chrome, status strip, WebSocket/state plumbing, build tooling — and treat
the robot-control panels as a clean-sheet rewrite against §2. Trying to force
arm-control widgets to mean something on a quadruped produces a UI that lies.

### 1.2 D1 Max is not a request/response API.

X2/Cadet-class stacks usually expose ROS 2 topics or a vendor HTTP API, where
one UI action maps to one call. The D1 Max does not work that way. It speaks
**ZSKJ over UDP 8082**, and it is a *stateful session* with liveness
requirements:

- **Handshake** (1000) opens an exclusive session. One client at a time.
- **Heartbeat** (1001) at **5 Hz, mandatory**. Miss it and the robot drops you.
- **Teleop** (1003) at **50 Hz while moving**, 5 Hz idle. A `Move()` expires
  after **1 second** — it is a dead-man switch, not a setpoint.
- **Ownership** can be preempted: the handset App can take control from the
  SDK; the SDK cannot take it back. Message 1016 arrives unsolicited.

**This means there is no stateless backend.** You cannot put the protocol
behind a lambda or a per-request handler. Forge needs a **resident session
daemon** that owns the socket and runs those timers independently of whether
any browser is connected. The HTTP/WS layer talks to the daemon, never to the
robot.

That daemon already exists in this repo as `tools/d1max_client.py`.

---

## 2. D1 Max control-plane spec

This is the ground truth the Forge backend has to implement. All of it is
already working in `tools/`.

### 2.1 Wire format

```
+---------+---------+---------+------------------+
| 4 bytes | 4 bytes | 4 bytes |     8 bytes      |
|  sync   | length  | msg_id  |    reserved      |   + UTF-8 JSON ASDU
| 5A534B4A|   LE    |   LE    |                  |
+---------+---------+---------+------------------+
```
`HEADER_LEN = 16`, `MAX_ASDU = 65535`, protocol `1.2.0`.
Transport: **UDP 8082** (default since v0.1.1). WebSocket 8081 also exists.

### 2.2 Message types

| ID | Name | Direction | Rate | Purpose |
|---|---|---|---|---|
| 1000 | HANDSHAKE | → | once | open session, get `sn` |
| 1001 | HEARTBEAT | ↔ | **5 Hz** | liveness, RTT |
| 1002 | COMMAND | → | on demand | discrete commands (§2.3) |
| 1003 | TELEOP | → | **50 Hz** active / 5 Hz idle | velocity, 1 s dead-man |
| 1004 | BODY_STATE | ← | 1 Hz | battery, mode, speed level, `ctrl_source` |
| 1005 | FAULT | ← | event | fault codes |
| 1008 | SENSOR_CONFIG | → | on demand | enable/disable streams |
| 1013 | TAKE_CONTROL | → | on demand | request ownership |
| 1015 | RELEASE_CONTROL | → | on demand | give up ownership |
| 1016 | CONTROL_TAKEN | ← | event | **you were preempted — hard stop** |
| 1017 | CONTROL_RELEASED | ← | event | ownership freed |
| 1018 | CAMERA_BITRATE | → | on demand | video tuning |
| 1050 | GOODBYE | → | once | clean close |
| 1100 | IMU | ← | stream | sensor 10 |
| 1101 | LUX | ← | stream | sensor 20 |
| 1102 | **MOTION** | ← | **50 Hz** | sensor 30 — **the odometry source** |
| 1103 | BODY_SPEED | ← | stream | sensor 40 (only one honouring `freq`) |
| 1104 | JOINT_STATE | ← | stream | sensor 50 |

Source IDs: `BODY=1`, `APP=2`, `SDK=3`, `EXTERNAL=4`.
Control holders: `NONE=0`, `APP=1`, `SDK=2`, `EXTERNAL=3`.
Handshake results: `OK=0`, `PROTOCOL_MISMATCH=10`, `ALREADY_CONTROLLED=20`.

**Message 1102 is the whole ballgame for missions and SLAM.** It carries
`position[3]`, `quat[4]` (documented `[w,x,y,z]`), `v_body[3]`, `v_world[3]`,
`omega_body[3]`, `time_stamp` (ns) — the control board's 50 Hz leg-kinematics +
IMU estimate. It is the only pose source; the robot publishes no `/odom`.

### 2.3 Command vocabulary

Every discrete button in Forge maps to one of these strings. Anything not on
this list must be rejected client-side — the robot silently ignores unknowns,
which reads as a dead button.

```
emergency/stop                    emergency/recover

action/stand_up                   action/crawl
action/lie_down                   action/locked
action/climb                      action/dsb
action/slim                       action/gait_walk
action/new1new  (wiggle)          reverse_head_tail

mode/general                      mode/in_place
mode/navigation                   mode/stair
mode/follow *                     mode/track *

speed/low                         speed/medium
speed/high

knee_mode/same_direction †        knee_mode/medial_facing †

fill_light/light_auto_work_on     fill_light/light_auto_work_off
fill_light/front_light_on         fill_light/front_light_off
fill_light/back_light_on          fill_light/back_light_off
```
\* not on ZSM-1 / ZSM-1F  † not on point-foot variants — gate these in the UI
off the model reported at handshake.

Full method-by-method API: **`docs/dev/08-sdk-api-reference.md`**.

### 2.4 Teleop scaling

`lx`, `ly`, `rx` are normalised **±1.0** and rescaled by the active speed level:

| Level | forward_back | left_right | yaw |
|---|---|---|---|
| **1 Low** | ±1.0 m/s | ±0.5 m/s | ±1.5 rad/s |
| **2 Medium** | ±2.0 m/s | ±0.5 m/s if \|fwd\| < 1.0, else **0** | ±1.5 rad/s if \|fwd\| < 1.0, else ±1.0 |
| **3 High** | ±3.0 m/s | ±0.5 m/s if \|fwd\| < 1.0, else **0** | ±1.5 / ±1.0 / **±0.5** above 2.0 m/s |

Two consequences for the UI. The same stick deflection means three different
things, so the active level must be visible next to the pad. And **lateral
translation is silently cut to zero above 1 m/s forward at Medium and High** —
a strafe input that works on the bench does nothing at speed.

The C++ SDK documents the order explicitly:
`Move(left_right, forward_back, yaw)` — **lateral first**, `+left_right` = translate
left, `+forward_back` = forward, `+yaw` = rotate left (`docs/source/3.3`).

> **Still verify on hardware.** What is documented is the *SDK method* order.
> `tools/d1max_proto.py` writes the wire fields `lx`/`ly`/`rx` directly, and the
> mapping from those field names onto the documented arguments is inferred, not
> stated. `tools/d1max_mission.py` ships an `AxisCalibration` routine that drives
> one axis and decomposes the resulting motion. **Run it before trusting any
> autonomous mode** — see `docs/dev/08-sdk-api-reference.md`.

---

## 3. Architecture

Three layers. The split matters because layer 1 must survive the browser
disconnecting.

```
┌──────────────────────────────────────────────┐
│  forge-ui        rebranded reference frontend│
│                  static, no robot knowledge  │
└───────────────┬──────────────────────────────┘
                │  REST + WebSocket (localhost)
┌───────────────▼──────────────────────────────┐
│  forge-server   FastAPI. Route names mirror  │
│                 agi-dashboard 1:1 where the  │
│                 semantics actually match.    │
│                 Owns mission + map state.    │
└───────────────┬──────────────────────────────┘
                │  in-process calls
┌───────────────▼──────────────────────────────┐
│  forge-core     RESIDENT session daemon      │
│                 • 5 Hz heartbeat thread      │
│                 • 50 Hz teleop pump          │
│                 • 350 ms watchdog → zero     │
│                 • ownership tracking         │
│                 = tools/d1max_client.py      │
└───────────────┬──────────────────────────────┘
                │  ZSKJ / UDP 8082
          ┌─────▼─────┐         ┌──────────────┐
          │  RK3588   │◄───────►│   Orin NX    │
          │  motion   │  wired  │  ROS 2 Humble│
          │ .234.1 /  │         │ .168.168.100 │
          │ .168.168  │         │ Zenoh :7447  │
          └───────────┘         └──────────────┘
```

### 3.1 What already exists and gets reused verbatim

| File | Role in Forge |
|---|---|
| `tools/d1max_proto.py` | wire codec, command table, validation. **Ships as-is.** |
| `tools/d1max_client.py` | the session daemon — heartbeat, teleop pump, ownership |
| `tools/d1max_mission.py` | waypoint record/replay, 20 Hz pure-pursuit, axis calibration |
| `tools/d1max_slam.py` | SLAM check/save, PCD → nav2 grid, pure Python |
| `tools/d1max_map.py` | SSH-driven mapping on the Orin — laptop needs no ROS 2 |
| `tools/d1max_odom_bridge.py` | 1102 → `/odom` + TF, runs on the Orin |
| `tools/d1max_sim.py` | protocol-accurate fake robot — **build the UI against this** |

The simulator is the reason this port can be built and demoed without the robot
present. Every route below can be exercised against `--sim`.

### 3.2 Route surface already implemented

`tools/d1max_console.py` serves these today. They are the starting set for
`forge-server`; names get aligned to the reference app once it is readable.

```
GET   /api/state              GET   /api/events   (SSE)
GET   /api/netcheck           GET   /api/trace
GET   /api/missions           GET   /api/maps
GET   /api/slam/check         GET   /api/map/preview/<name>
POST  /api/connect            POST  /api/disconnect
POST  /api/velocity           POST  /api/stop
POST  /api/command            POST  /api/estop
POST  /api/take_control       POST  /api/release_control
POST  /api/sensor             POST  /api/slam/save
POST  /api/record/start       POST  /api/record/mark
POST  /api/record/undo        POST  /api/record/finish
POST  /api/record/cancel      POST  /api/calibrate
POST  /api/mission/run        POST  /api/mission/abort
POST  /api/mission/pause      POST  /api/mission/resume
POST  /api/mission/delete
```

`/api/velocity` is refused while a mission is `RUNNING` — a mission and a human
must never both be writing teleop.

### 3.3 Transport choice: SSE → WebSocket

The existing console pushes state over **SSE**. `robo-teleop` uses a
**WebSocket** (`@app.websocket("/ws")`). If the reference app uses a WebSocket —
likely, given the house pattern — switch Forge to WS so the frontend's existing
socket code ports unchanged. The daemon-side change is small: the state
broadcaster already produces a JSON-safe dict via `snapshot()`.

---

## 4. Reference-app mapping (fill on unblock)

The D1 Max column is already fixed. The left column gets populated by reading
`szwedk/agi-dashboard` once it is reachable.

| agi-dashboard (X2/Cadet) | Robolink Forge (D1 Max) | Port action |
|---|---|---|
| _connect / pair flow_ | handshake 1000 + resident session | rewrite — session, not request |
| _telemetry poll or topic sub_ | 1004 @ 1 Hz + 1102 @ 50 Hz | rewrite |
| _velocity / cmd_vel_ | 1003 @ 50 Hz + 1 s dead-man | rewrite — add the watchdog |
| _e-stop_ | `emergency/stop` + local pump zero | keep UI, swap call |
| _arm / hand / waist panels_ | **no equivalent** | delete |
| _head pan-tilt_ | `ControlHead` — In-Place mode only | keep, gate on mode |
| _posture / stance_ | `action/*` (§2.3) | remap vocabulary |
| _gait / walk mode_ | `mode/*` + `speed/*` | remap vocabulary |
| — | `knee_mode/*`, fill lights, `reverse_head_tail` | **new panels** |
| _map / nav view_ | odom-frame trace + nav2 grid from `d1max_slam.py` | reuse existing |
| _mission / task runner_ | `d1max_mission.py` waypoints | reuse existing |
| _camera_ | 1018 bitrate; stream path TBD | investigate |
| _branding, theme, layout, chrome_ | Robolink Forge / Robo Inc | rebrand only |

---

## 5. Rebrand map

| Reference | Forge |
|---|---|
| AGI Dashboard | **Robolink Forge** |
| AgiBot / vendor marks | **Robo Inc** |
| `agi-dashboard`, `agi_dashboard` | `robolink-forge`, `robolink_forge` |
| `agi`, `AGI` prefixes | `forge`, `Forge` |
| X2 / Cadet model strings | D1 Max, ZSM-1, ZSM-1F |
| package name / bundle id | `com.roboinc.robolinkforge` |
| default port | keep the reference's, to preserve muscle memory |

Do this with an explicit rename table checked into the repo, not a blind
`sed -i`. Blind renames on a robotics codebase hit ROS topic names, URDF link
names, and saved-config keys, and the breakage shows up at runtime on hardware.

---

## 6. Phases

**P0 — Unblock and read** *(blocked on you)*
Mirror the repo under `feraco`, I attach it, and produce the real §4 table plus
a file-by-file port inventory. Nothing below is safe to start before this —
choosing a stack blind is how you end up rewriting it twice.

**P1 — Scaffold** *(1 day after P0)*
Fork the reference structure, apply §5, rip out the humanoid panels, get it
building and serving with all robot calls stubbed.

**P2 — Wire the control plane** *(2–3 days)*
Mount `forge-core`. Connect, heartbeat, teleop pad, e-stop, `action/*`,
`mode/*`, `speed/*`, lights, ownership. **Built entirely against
`d1max_sim.py`.** Exit criterion: full UI exercise with no robot present.

**P3 — Hardware bring-up** *(0.5 day, needs the robot)*
Connect to the hotspot. **Run axis calibration first.** Verify every command
string against the real robot — the simulator accepts things hardware may not.
Confirm the preemption path: take control from the handset app and check the
UI hard-stops on 1016.

**P4 — Missions and map** *(2–3 days)*
Port the mission recorder and map canvas. Record-by-driving → waypoint list →
pure-pursuit replay. Nav2 grid rendering from `d1max_slam.py`.

**P5 — SLAM integration** *(2 days)*
Surface `d1max_map.py`'s five-command SSH workflow in the UI: doctor → record →
build → fetch. Keep the CLI as the fallback; it is what actually gets used in
the field when the UI is wrong about something.

**P6 — Packaging**
Follow `robo-teleop`'s installer pattern — one script, one reboot, one URL.

---

## 7. Risks

| Risk | Impact | Mitigation |
|---|---|---|
| Reference app is a stack we don't want (heavy SPA, vendor cloud coupling) | P1 slips into a rewrite | Read it in P0 before committing to a port |
| **`lx`/`ly` axes unconfirmed** | robot drives the wrong way under autonomy | `AxisCalibration` in P3, gate P4 on it |
| Handset app preempts mid-mission | robot keeps its last velocity | 1016 → hard stop is already in `d1max_client.py`; test it in P3 |
| Wi-Fi drop during teleop | 1 s dead-man zeroes it — by design | surface link RTT prominently |
| 1102 dead reckoning drifts | long missions wander | close loops; relocalisation is not built yet |
| OTA / reflash **wipes the Orin NX** | maps and SLAM install lost | `docs/dev/07`, §8 — back maps up off-robot |
| `librobot_sdk.so` is glibc + libstdc++ with no unmangled exports | **no mobile port of the vendor SDK** | Forge speaks the wire protocol directly, so mobile stays open |

---

## 8. Decisions needed from you

1. **Mirror `agi-dashboard` under `feraco`?** — P0 cannot start otherwise.
2. **Web-only, or web + native?** The protocol is reimplemented in pure Python,
   so a native client is viable later; the vendor `.so` is not.
3. **Keep the reference's stack, or standardise on the `robo-teleop` pattern**
   (FastAPI + WS + single-file static)? The second is already proven twice in
   your org and is what the D1 Max console is written in today.

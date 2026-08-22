# 05 — Mission System Design

A "mission" — drive this route, stop here, look at that, come back, recharge — has
no representation anywhere in the shipped platform. There is no waypoint concept,
no goal pose, no task queue. This document proposes the design.

## Where the executor runs

**On the Orin NX. Not on the phone, not on the laptop.**

The reasoning is not architectural taste, it is the failure model. Consider a
mission executor running on a phone, driving `Move()` over Wi-Fi:

- Operator walks around a corner → Wi-Fi degrades → control loop stalls → the
  robot's 1-second command expiry stops it mid-mission, in an arbitrary place.
- Phone rings / app backgrounds / battery dies → same outcome.

Now on the Orin NX: the executor is on the robot's own wired LAN to the RK3588
(`192.168.168.100` → `192.168.168.168`), on a link that cannot degrade
independently of the robot. Wi-Fi loss becomes "the operator temporarily can't
watch", not "the robot is uncommanded". The client becomes a thin shell that
authors, uploads, starts, monitors and aborts missions.

This also matches the architecture the vendor's own autonomous-recharge demo
implies, which takes both endpoints on its command line
(`./control 192.168.168.168 8081 192.168.168.100 10010` — `docs/source/5.2`).

```
  Client (Linux / Android / iOS)
    │  author · upload · start · monitor · abort
    │  WebSocket or gRPC, tolerant of loss
    ▼
  ┌────────────────────────────── Orin NX ──────────────────────────────┐
  │                                                                      │
  │  mission-server ──▶ mission-executor ──▶ controller ──┐              │
  │       │                    │                          │              │
  │       │              localisation                     │ 50 Hz        │
  │       │                    ▲                          │ teleop 1003  │
  │       │              SLAM / map                       │              │
  │       │                    ▲                          │              │
  │       └──── state ─────────┴── /front_lidar, /odom    │              │
  │                                                        │             │
  └────────────────────────────────────────────────────────┼─────────────┘
                                                           │ UDP 8082
                                                    ┌──────▼──────┐
                                                    │   RK3588    │
                                                    └─────────────┘
```

**The client must never be in the control loop.** It sits alongside it, observing.

## Mission schema

A mission is a versioned document, portable between clients and robots, bound to a
specific map.

```jsonc
{
  "schema_version": 1,
  "id": "b3f1…",
  "name": "Perimeter inspection — Level 2",
  "map_id": "level2-2026-08-14",       // missions are meaningless without a map
  "created_by": "…",

  "defaults": {
    "speed_level": "low",              // low | medium | high — pin it
    "mode": "general",
    "goal_tolerance_m": 0.25,
    "heading_tolerance_rad": 0.20
  },

  "steps": [
    { "type": "posture",  "action": "stand_up" },
    { "type": "set_mode", "mode": "general" },
    { "type": "set_speed","level": "low" },

    { "type": "goto",
      "pose": { "x": 12.4, "y": -3.1, "theta": 1.57 },
      "tolerance_m": 0.25,
      "timeout_s": 120 },

    { "type": "wait", "seconds": 5 },

    { "type": "capture",
      "camera": "front",
      "label": "valve-array-3" },

    { "type": "light", "target": "front", "on": true },

    { "type": "goto",
      "pose": { "x": 18.0, "y": 4.2, "theta": 0.0 },
      "posture_hint": "stair",         // approach expects stairs
      "timeout_s": 180 },

    { "type": "posture", "action": "lie_down" }
  ],

  "on_failure":   "abort_and_hold",    // abort_and_hold | return_to_start | continue
  "on_low_battery": { "threshold_pct": 25, "action": "return_to_start" },
  "on_control_lost": "abort"           // not configurable in practice — always abort
}
```

### Step types

| Type | Purpose |
|---|---|
| `goto` | Drive to a pose in the map frame. The core primitive |
| `posture` | `stand_up`, `crawl`, `lie_down`, `locked`, `climb`, `slim`, `gait_walk`, `dsb` |
| `set_mode` | `general`, `in_place`, `stair` |
| `set_speed` | `low`, `medium`, `high` |
| `wait` | Dwell |
| `light` | Fill lights — pairs naturally with `capture` and with the `1101` lux reading |
| `capture` | Grab a camera frame, tag it, store it |
| `look` | Head pan/tilt via in-place `rx`/`ry` |
| `loop` | Repeat a step range N times or until a condition |

Keep the vocabulary small at first. `goto` + `posture` + `wait` + `capture` covers
the large majority of real inspection work, and every step type you add is one more
thing that can fail in the field.

## The `goto` controller

This is where the real engineering is. `goto` consumes a pose from localisation and
emits teleop at 50 Hz.

**Controller: pure pursuit.** It's simple, well understood, tolerant of pose noise,
and produces smooth motion. Sophisticated alternatives (MPC, DWA) are not worth
their tuning cost until pure pursuit demonstrably fails.

**Output mapping** — remember `Move()` takes normalised percentages whose meaning
depends on the speed level ([doc 01](01-platform-architecture.md)):

```
v_cmd (m/s)  →  forward_back  =  clamp(v_cmd / v_max[speed_level], -1, 1)
ω_cmd (rad/s)→  yaw           =  clamp(ω_cmd / ω_max[speed_level], -1, 1)
```

Pin the speed level for the whole mission so `v_max` is constant. At medium and
high levels, yaw and lateral authority are *cut* as forward speed rises — a
controller that assumes constant authority will understeer exactly when it's moving
fastest. If you must use higher speeds, model the reduction explicitly.

Prefer **low** speed for autonomous operation. 1 m/s with full 1.5 rad/s yaw
authority is a far more controllable plant than 3 m/s with 0.5 rad/s, and nothing
about inspection work needs 3 m/s.

**Reactive safety layer**, running underneath the planner and able to override it:

- Ultrasonics (`/uss_driver/uss_{left,right}/range`, 0–4 m, 10 Hz) — close-range
  lateral obstacles the LiDAR may miss.
- Rear LiDAR — anything behind, especially when reversing.
- Front LiDAR — a hard stop volume ahead, independent of the planner.

Everything the planner does is a suggestion; this layer holds a veto.

## Executor state machine

```
IDLE ──load──▶ READY ──start──▶ RUNNING ──▶ COMPLETED
                 ▲                │  │
                 │            pause│  │fault / control lost / timeout
                 │                ▼  ▼
                 └───resume──── PAUSED   ABORTED
                                          │
                                    hold position,
                                    zero velocity,
                                    optionally lie down
```

Abort triggers, all mandatory:

- `1016` control taken by the App → **immediate abort**. Non-negotiable and not
  configurable.
- Any `FatalError` in `1005`.
- Soft or hard e-stop asserted (visible in `1004`).
- Localisation confidence below threshold — a lost robot must stop, not guess.
- Step timeout exceeded.
- Battery below threshold → `return_to_start` or dock.
- Operator abort from any client.

**Abort must be idempotent and reachable from every state**, including while the
executor itself is unhealthy. The simplest robust implementation is a separate
watchdog process holding its own connection to the RK3588, able to assert
`emergency/stop` even if the executor has hung. That redundancy is cheap and it is
the difference between a bug and an incident.

## Client protocol

Between client and `mission-server`, use something loss-tolerant and
reconnect-friendly — WebSocket with JSON, or gRPC. Not the robot's `ZSKJ` protocol;
that's for the RK3588 link only.

```
POST   /missions              upload
GET    /missions              list
POST   /missions/{id}/start
POST   /missions/{id}/abort
POST   /missions/{id}/pause | /resume
GET    /missions/{id}/state   → current step, pose, progress, faults
WS     /telemetry             → pose @ 10 Hz, state, faults, battery
GET    /maps                  list
GET    /maps/{id}/grid        2-D occupancy grid (PNG + metadata YAML)
GET    /maps/{id}/cloud       decimated PCD, on demand only
```

Rate-limit and decimate everything on this interface. A phone gets a 2-D grid and a
10 Hz pose, never a 96-line point cloud at 10 Hz.

**Reconnect must be free of side effects.** A client that drops and returns
re-subscribes to state; it does not restart, resume or alter the mission. State
lives on the robot.

## Mission authoring UX

**On desktop:** map view, click to place waypoints, drag to reorder, per-step
property panel, simulated preview.

**On mobile:** authoring on a phone is mostly a poor experience — but two mobile
workflows are genuinely valuable and worth building properly:

1. **Record-by-driving.** Teleop the robot along a route and drop a waypoint at the
   current pose with one tap. This is by far the most natural way to author a
   mission on this hardware, it requires no map-reading skill, and it produces
   waypoints that are known-reachable *because the robot just reached them*. If you
   build one authoring feature, build this one.
2. **Run and monitor.** Pick a saved mission, start it, watch progress, abort. This
   is the common case in the field, and it's the one that most justifies a phone
   app at all.

Full graph-editing on a 6-inch screen is not worth building.

## Persistence and portability

- Missions and maps are documents with stable IDs; a mission references a `map_id`
  and is invalid without it.
- **Back everything up off-robot.** `docs/source/1.6` warns that reflashing or an
  OTA update wipes the Orin NX. Maps represent hours of fieldwork.
- Version the schema from day one (`schema_version` is in the example above).
  Missions authored today should still load after a year of development.
- Store run history — timestamps, per-step outcomes, faults, captured images.
  Inspection work is only useful if the results are comparable across runs, and
  retrofitting this later means throwing away the runs you already did.

## Open questions to settle on hardware

These need answers from the physical robot before the design can be finalised:

1. **Does `mode/navigation` do anything?** It exists in the protocol
   ([doc 02](02-wire-protocol.md)) but is absent from the C++ SDK. If it engages a
   vendor navigation behaviour, that changes the plan substantially.
2. **What is actually on port 10010 of the Orin NX?** `docs/source/5.2` invokes the
   recharge demo against it. If there's an existing service there, it may already
   provide docking, and possibly more.
3. **Is a vendor SLAM node already running?** `ros2 node list` and `robot-launch`
   will say. Integrating beats reimplementing.
4. **Are the two boards' clocks disciplined to a common source?** Determines how
   much work the odometry bridge's time-sync logic needs
   ([doc 03](03-slam-mapping-plan.md)).
5. **Do `lx`/`ly` map to `forward_back`/`left_right` as inferred?** The
   documentation is self-contradictory. Test at low speed with a hand on the
   e-stop.
6. **How badly does gait vibration affect the LiDAR IMU?** Decides FAST-LIO2 vs.
   Point-LIO.

Answer 1–3 in the first session with the robot; they're all read-only commands and
they can each save weeks.

# 09 — Developing Obstacle Avoidance and Mapping Yourself

`docs/dev/08` establishes that the SDK ships neither. This is how to build them.

The short version: the robot gives you **sensors and a velocity input**, and
everything between those two is yours to write. That is a normal amount of work
for a mobile robot — nav2 does most of it — but it depends on one thing the
robot does not provide, and one seam that had to be built.

---

## The seam: two bridges

A nav stack needs a pose in and a velocity out. The D1 Max publishes neither
on ROS 2 — pose lives in a UDP message, velocity input is a UDP message. So
both directions are bridges:

```
  LiDAR ─┐                                       ┌─ /cmd_vel
  IMU  ──┼─→ ROS 2 (Orin NX) ─→ SLAM ─→ nav2 ─→──┤
  USS  ──┘         ▲                             │
                   │ /odom + TF                  ▼
        d1max_odom_bridge.py            d1max_cmdvel_bridge.py
                   ▲                             │
                   └────── UDP 8082 ─────────────┘
                          RK3588 control board
```

| | File | Direction |
|---|---|---|
| pose in | `tools/d1max_odom_bridge.py` | msg 1102 @ 50 Hz → `/odom` + `odom→base_link` |
| velocity out | `tools/d1max_cmdvel_bridge.py` | `/cmd_vel` → msg 1003 teleop |

With both running, **the robot looks like a normal ROS 2 diff/omni base** and
the rest of the stack is off-the-shelf.

### What the cmd_vel bridge handles for you

- **Unit conversion.** nav2 speaks m/s; the robot wants normalised ±1.0
  rescaled by speed level. The bridge converts, and logs the real limits to
  put in your planner config — the most common cause of an oscillating nav
  stack is planning for a velocity the robot will never produce.
- **The conditional ceilings.** At MEDIUM/HIGH, lateral is cut to **zero**
  above 1 m/s forward and yaw drops in steps. A planner unaware of this
  issues commands that silently do nothing, which looks exactly like the
  robot ignoring obstacles. Default is level 1 (LOW), where all three axes
  keep full authority.
- **A safety gate** mirroring `MissionExecutor._abort_reason()` — one
  definition of "unsafe to drive", so the waypoint executor and the nav stack
  cannot disagree.
- **Not defeating the dead-man.** The robot's 1 s `Move` expiry and the
  client's 350 ms watchdog both stay in force. **If the planner crashes, the
  robot stops on its own.** That is the most valuable safety property in the
  stack — never paper over it with a keep-alive.

```bash
ssh robot@192.168.168.100
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=24 RMW_IMPLEMENTATION=rmw_zenoh_cpp
python3 d1max_cmdvel_bridge.py --host 192.168.168.168 --speed-level 1
```

---

## Phase 0 — Settle the axes. Everything else is blocked on this.

The wire fields are `lx`/`ly`/`rx`. This repo assumes `lx` = forward. The C++
SDK's `Move(left_right, forward_back, yaw)` puts **lateral first**. Both cannot
be right, and no amount of reading resolves it — the protocol PDF and the
header disagree.

```bash
python3 tools/d1max_console.py --host 192.168.234.1   # CALIBRATE button
```

It drives one axis briefly and decomposes the resulting motion against the body
heading. If it reports the axes transposed, run the bridge with `--swap-xy`.

**Do not skip this and do not run autonomy before it passes.** A transposed
axis means every avoidance manoeuvre drives into what it was avoiding.

Same session, settle the other undocumented encoding: provoke a real fault
(unplug a LiDAR, run the battery down) and record what `level` actually
contains. `FATAL_FAULT_LEVEL = 3` in the bridge matches what
`d1max_mission.py` already assumed, but the C++ `FaultLevel` enum runs the
*other* way (`FatalError=1`, `Error=2`, `Warn=3`). If the wire uses the C++
encoding, both files currently abort on warnings and **ignore fatal errors**.
Cheap to check, expensive to be wrong about.

## Phase 1 — Reactive safety, before any autonomy

Build this first, run it always, keep it in its **own process**.

A planner is a best-effort optimiser. A safety layer is a guarantee, and the
two must not share a failure mode. This one does nothing but read LiDAR and
zero the setpoint:

- Subscribe `/front_lidar` and `/rear_lidar` (`PointCloud2`, 10 Hz,
  **best_effort** — match that QoS or you will receive nothing).
- Crop to a body-frame box in the direction of travel, above the ground plane
  and below robot height.
- Nearest return inside the box under threshold → command zero, latch, require
  an explicit clear.

10 Hz gives 100 ms of latency before you even start reacting. At the bridge's
default 0.40 m/s cap that is 4 cm of travel per scan — fine. At HIGH speed
(3 m/s) it is 30 cm per scan, which is why the bridge defaults to LOW.

**The ultrasonics do not help here.** There are only two, left and right —
**no forward sensor** — so they cover flanks, not travel direction. Useful for
squeeze detection and wall-following, not collision avoidance. And the driver
leaves `field_of_view`, `min_range` and `max_range` **fixed at 0**
(`docs/source/4.3`), so you must hardcode the cone from the datasheet; anything
consuming those fields as published will compute nonsense.

## Phase 2 — Mapping

Already built. `docs/dev/07` is the how-to:

```bash
python3 tools/d1max_map.py doctor
python3 tools/d1max_map.py record lab
python3 tools/d1max_map.py build lab      # FAST-LIO2 on the Orin
python3 tools/d1max_map.py fetch lab      # → map.pcd, map.pgm, map.yaml
```

FAST-LIO2 on the front RoboSense + its internal IMU. The IMU is *inside* the
LiDAR housing, so the extrinsic is near-identity — this is the easy pairing.
`lidar_type: 2` (Velodyne-style), **not** the `1` most FAST-LIO examples show.

## Phase 3 — Localisation. This is the real missing piece.

Mapping works; **relocalisation does not exist yet**, and without it missions
run in an odometry frame that resets every boot. A route recorded today is
meaningless tomorrow. This is what makes routes repeatable, and it is the
highest-value thing left to build.

Two options:

| Approach | Notes |
|---|---|
| **AMCL on the 2-D grid** | Standard, well understood. Needs a `LaserScan`, so flatten the PointCloud2 with `pointcloud_to_laserscan`. Throws away the 3-D structure. The robot's install space already contains a `laser_scan` package — check whether it does this for you. |
| **Scan-to-map against `map.pcd`** | Keeps 3-D, better on stairs and slopes, more work to integrate. |

Start with AMCL. It gets `map → odom` published, which is all nav2 needs, and
the drift you are correcting is legged dead reckoning — not subtle.

## Phase 4 — nav2

Once `map → odom → base_link` is complete and `/cmd_vel` drives the robot,
this is configuration rather than development.

- **Costmap input:** `PointCloud2` via the voxel layer, `observation_sources`
  pointed at both LiDARs. Set `min_obstacle_height` above the ground plane —
  the single most common cause of a robot that thinks the floor is a wall.
- **Controller:** DWB or MPPI. Set `max_vel_x` / `max_vel_y` / `max_vel_theta`
  to exactly what the bridge logs at startup.
- **Footprint:** the D1 Max is not round. Use a polygon, and remember `Slim()`
  changes it.
- **Recovery behaviours:** the spin recovery assumes yaw authority the robot
  may not have at speed. Pin LOW.

## Phase 5 — Traversability, where this robot is actually different

A binary occupancy grid discards the D1 Max's whole advantage. It climbs 25 cm
steps and crosses 80 cm obstacles; a 2-D costmap marks both as walls and plans
around them.

The interesting work is a cost layer over the 3-D map keyed on **local slope
and step height** rather than occupancy — cheap to traverse, expensive,
impossible — and switching `mode/stair` on when the planned path crosses a
step band. Nothing else in this stack is novel; this is.

---

## Where it runs

**On the Orin NX, all of it.** It already has Humble, Zenoh on `:7447`, and
both LiDARs. Running the loop over Wi-Fi puts a radio link inside your control
loop — the robot's internal protection already auto-stops on network
congestion, so a marginal link produces a robot that stops constantly for no
visible reason.

Two standing hazards:

- **OTA or reflash wipes the Orin** (`docs/source/1.6`). Maps are hours of
  fieldwork and your nav config is hours more. Keep both in git, deploy with a
  script, and back up `~/.d1max/maps` off-robot.
- **Do not start a Zenoh router on the Orin.** One is already running;
  `Address already in use` on `:7447` means the robot is healthy.

## Order of work

| | Depends on | Effort |
|---|---|---|
| 0. Axis + fault-level verification | hardware | hours |
| 1. Reactive safety layer | 0 | 2–3 days |
| 2. Mapping | — | **done** |
| 3. Relocalisation | 2 | 3–5 days |
| 4. nav2 bring-up | 1, 3 | 3–5 days |
| 5. Traversability layer | 4 | open-ended |

Phases 0–1 are worth doing even if you never build the rest: a reactive stop
makes every other mode of operation safer, including manual driving.

## The one thing not to do

Do not put avoidance in the planner alone and call it done. The planner runs
at whatever rate it manages, from a costmap that is always slightly stale,
using a model of the robot that is always slightly wrong. The independent
Phase 1 layer is what stands between that and a 41 kg robot. Keep them
separate, and keep the dead-man intact underneath both.

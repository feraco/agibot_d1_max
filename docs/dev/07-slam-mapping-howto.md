# 07 — Building and Saving a SLAM Map

## The short way — no ROS 2 on your laptop

The Orin NX already runs ROS 2 Humble and already sees both LiDARs. Drive it
from your laptop over SSH and you never install ROS at all:

```bash
sudo apt install -y openssh-client sshpass    # once
conda deactivate                              # if conda is active

python3 tools/d1max_map.py doctor             # reachable? what is running?
python3 tools/d1max_map.py record lab         # records while you drive
python3 tools/d1max_map.py install-slam       # once, builds FAST-LIO2 on the robot
python3 tools/d1max_map.py build lab          # SLAM over the recording
python3 tools/d1max_map.py fetch lab          # pull it back + 2-D grid
```

`record` needs nothing beyond what the robot already has, so you can capture
data on your first session and decide how to process it later. LiDAR time on a
real site is the expensive part; a bag can be re-processed as often as you like.

While recording, drive from the console in another window. Technique matters
more than tuning: speed LOW, gentle inputs, slow turns, **close the loop**, and
cover walls, corners and doorways.

The rest of this document is the manual route — useful when you want ROS 2 on
your own machine, or when something above fails and you need to see the parts.

---

## Three things that will bite you on the Orin NX

**1. Do not start a Zenoh router on the Orin.** It already runs one. Starting a
second gives:

```
Unable to open listener tcp/[::]:7447: Address already in use (os error 98)
```

That error means the robot is healthy. On the Orin you only need:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic list
```

`ros2 run rmw_zenoh_cpp rmw_zenohd` is for a **remote laptop** only.

**2. `handshake rejected: already controlled by another terminal`.** Something
else holds the control session — usually the RC handset app. The odometry
bridge only ever reads telemetry, but the handshake is refused regardless.
Close the RC app (fully, not just backgrounded), or close the operator console
if it is connected. `--external` identifies as EXTERNAL rather than SDK and is
worth a try. Also note that **from the Orin, the control board is
`192.168.168.168`**, not the Wi-Fi address.

**3. The LiDARs are RoboSense, not Livox.** The robot's own install space
carries `rslidar_sdk` and `rslidar_msg`. RoboSense clouds are laid out
Velodyne-style (`ring` + per-point `timestamp`), so FAST-LIO needs
`lidar_type: 2`, not the `lidar_type: 1` that most FAST-LIO examples show.
`d1max_map.py build` reads the actual PointCloud2 field names off the running
topic and picks for you; override with `--lidar-type`.

The robot's driver install space also contains `uss_driver`, `imu_driver`,
`sixents_gps_driver`, `uwb_driver`, `nlink_parser`, `laser_scan`,
`realsense_ros_wrapper` and `livox_driver` — worth a look, since `laser_scan`
suggests a 2-D scan conversion may already exist.

---

Step-by-step for producing a map of a real space with the D1 Max.

The robot publishes LiDAR and IMU on ROS 2 from the Orin NX, but **no odometry,
no `/tf`, no map and no SLAM node** — see
[doc 03](03-slam-mapping-plan.md). This walks through filling those gaps.

Two things are worth separating up front:

| | Needs ROS 2? | What you get |
|---|---|---|
| **Odometry-frame routes** | No | Record and replay missions today, in the console. Drifts. |
| **SLAM map** | Yes | A metric map you can relocalise against and reuse across sessions. |

The console's mission recorder already works without any of this. Do that first
if you just want the robot driving itself — everything below is for a
persistent map.

---

## What you need

- Ubuntu 22.04 with **ROS 2 Humble**
- `ros-humble-rmw-zenoh-cpp` (the robot's middleware)
- A LiDAR-inertial SLAM front end — **FAST-LIO2** recommended, Point-LIO if
  gait vibration proves troublesome

Run it on your laptop for bench work. Run it on the Orin NX for anything real,
so mapping doesn't depend on Wi-Fi.

---

## Step 1 — Talk to the robot's ROS 2 graph

```bash
sudo apt-get install ros-humble-rmw-zenoh-cpp
```

Point the Zenoh router at the Orin NX. Edit:

```
/opt/ros/humble/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5
```

and set `connect: { endpoints: ["tcp/192.168.168.100:7447"] }`.

Set your laptop's IP to `192.168.168.x` (`x` not 100, 168 or 255). On Wi-Fi, add
the route first:

```bash
sudo ip route add 192.168.168.0/24 via 192.168.234.1
```

**Terminal A** — the router, which must stay running:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Verify from a second shell with the same three exports:

```bash
ros2 daemon stop && ros2 daemon start
ros2 topic list          # expect /front_lidar, /front_lidar/imu, …
ros2 topic hz /front_lidar   # expect ~10 Hz
```

---

## Step 2 — Publish odometry

This is the missing piece. `d1max_odom_bridge.py` connects to the RK3588 over
UDP, enables the 50 Hz motion stream (message 1102), and republishes it as
`nav_msgs/Odometry` plus the `odom → base_link` transform.

**Terminal B**, same three exports:

```bash
python3 tools/d1max_odom_bridge.py --host 192.168.234.1     # Wi-Fi
python3 tools/d1max_odom_bridge.py --host 192.168.168.168   # wired
```

Check it:

```bash
ros2 topic hz /odom        # ~50 Hz
ros2 run tf2_tools view_frames
```

The bridge prints the observed offset between the robot's clock and this
machine's every 5 seconds. Watch it — a large or drifting offset degrades map
quality in ways that are hard to diagnose later. `--use-robot-clock` stamps
with the robot's own nanosecond timestamp instead.

**Publish the robot model too**, so sensor extrinsics come from the URDF rather
than being hardcoded:

```bash
cd urdf/max_description && colcon build && source install/setup.bash
ros2 launch max_description display_launch.py
```

---

## Step 3 — Check you're ready

```bash
python3 tools/d1max_slam.py check
```

It reports the distro, `ROS_DOMAIN_ID`, RMW, which mapping topics are visible,
and what to fix. The console's **SLAM MAPS** panel shows the same thing, with a
RE-CHECK button.

Expect `READY TO MAP` before going further.

---

## Step 4 — Record a bag first

Do this before tuning anything. Being able to replay a traverse instead of
walking a 41 kg robot around again is the single biggest multiplier on
development speed here.

```bash
python3 tools/d1max_slam.py record --name lab-floor2
# Ctrl-C to stop; lands in ~/.d1max/maps/lab-floor2/bag
```

Capture a few different traverses — a closed loop, stairs, outdoors, cluttered
indoors.

---

## Step 5 — Run SLAM

Install FAST-LIO2 (once):

```bash
mkdir -p ~/lio_ws/src && cd ~/lio_ws/src
git clone https://github.com/Ericsii/FAST_LIO --recursive     # ROS 2 port
cd ~/lio_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash
```

Config for the D1 Max front LiDAR. The IMU is **inside** the LiDAR, so the
extrinsic between them is small and fixed — which is exactly why this pairing
is the easy one:

```yaml
common:
    lid_topic:  "/front_lidar"
    imu_topic:  "/front_lidar/imu"
preprocess:
    lidar_type: 1          # 1 = generic PointCloud2; match your driver
    scan_line:  96         # Airy 96-line
    blind:      0.5        # ignore returns closer than this (robot's own body)
mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    extrinsic_T: [ 0.0, 0.0, 0.0 ]     # LiDAR->IMU: same housing
    extrinsic_R: [ 1,0,0, 0,1,0, 0,0,1 ]
publish:
    path_en: true
pcd_save:
    pcd_save_en: true      # <- required, this is what you save at the end
    interval: -1           # one file for the whole run
```

The LiDAR-to-body extrinsic (404.3, 0, −37.7 mm from BASE, per
`docs/source/2.10`) belongs in your TF tree via the URDF, not in this file.

**Terminal C:**

```bash
ros2 launch fast_lio mapping.launch.py config_file:=d1max.yaml
```

Watch it in RViz — fixed frame `camera_init` or `map`, add `/cloud_registered`.

---

## Step 6 — Drive the space

This part is technique, and it matters more than the tuning:

- **Slow.** Speed Low, gentle inputs. Use the console's hold-to-drive pad.
- **Smooth turns.** Rotating fast in place is the most common way to break a
  LiDAR-inertial front end.
- **Close the loop.** Return to where you started. Without loop closure,
  FAST-LIO2 drifts on long traverses.
- **Cover the volume.** Walls, corners, doorways. Featureless corridors are the
  hard case; give them something to lock onto by including doorways and
  junctions.
- **Watch the cloud in RViz.** If it starts smearing, stop and restart — a bad
  map does not repair itself.

---

## Step 7 — Save the map

FAST-LIO2 writes its cloud to `~/lio_ws/src/FAST_LIO/PCD/scans.pcd` when
`pcd_save_en: true`. Then:

```bash
python3 tools/d1max_slam.py save --name lab-floor2 --pcd ~/lio_ws/src/FAST_LIO/PCD/scans.pcd
```

Or paste that path into **SAVE MAP** in the console's SLAM MAPS panel.

Either way you get, in `~/.d1max/maps/lab-floor2/`:

| File | What it is |
|---|---|
| `map.pcd` | the 3-D cloud, ground truth |
| `map.pgm` | 2-D occupancy grid, nav2 / map_server format |
| `map.yaml` | resolution and origin for the grid |
| `preview.png` | what the console shows |
| `meta.json` | ground plane, z band, bounds, point and cell counts |

The projection is **pure Python** — no PCL, no numpy, no ROS. It estimates the
floor as a low percentile of z, keeps a height band above it (0.15–1.20 m by
default, which clears the floor and the overhangs the robot walks under) and
marks a cell occupied at 2 hits or more.

Tune it without re-mapping:

```bash
python3 tools/d1max_slam.py grid --name lab-floor2 --res 0.03 --z-min 0.2 --z-max 1.5
python3 tools/d1max_slam.py list
```

**Symptoms and fixes**

| Looks like | Try |
|---|---|
| Solid blob | `--z-min` too low, floor is leaking in. Raise it. |
| Almost empty | `--min-hits 1`, or widen the z band |
| Ceiling arcs | lower `--z-max` |
| Sparse walls | lower `--min-hits`, or `--res 0.10` |

---

## Step 8 — Back it up

`docs/source/1.6` warns that reflashing or an OTA update **wipes the Orin NX**.
A map is hours of fieldwork.

```bash
tar czf lab-floor2-$(date +%F).tar.gz -C ~/.d1max/maps lab-floor2
```

---

## What this does not do yet

- **Relocalisation.** Missions currently run in the odometry frame, which
  resets every time the robot boots. Localising against a saved map is the next
  piece, and it's what makes routes repeatable across sessions
  ([doc 03](03-slam-mapping-plan.md), phase 3).
- **Planning.** The grid is nav2-format, so nav2 is the obvious next step, but
  nothing wires it up yet.
- **Traversability.** A binary occupancy grid throws away this robot's real
  advantage — it climbs 25 cm steps and crosses 80 cm obstacles. A slope and
  step-height cost layer over the 3-D map is where that lives.

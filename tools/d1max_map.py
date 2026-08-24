#!/usr/bin/env python3
"""One-command mapping for the D1 Max, driven over SSH to the Orin NX.

Your laptop does not need ROS 2. The Orin NX already has Humble, already sees
both LiDARs, and is the board the vendor documentation says user applications
belong on. This tool drives it from here.

    python3 tools/d1max_map.py doctor            # can we reach it? what's there?
    python3 tools/d1max_map.py record lab        # record while you drive
    python3 tools/d1max_map.py install-slam      # one-time: build FAST-LIO2
    python3 tools/d1max_map.py build lab         # offline SLAM -> cloud
    python3 tools/d1max_map.py fetch lab         # pull it back + 2-D grid

`record` needs nothing installed beyond what the robot already has, so you can
capture data today and decide how to process it later. That is deliberate:
LiDAR time on a real site is the expensive part, and a bag can be re-processed
as many times as you like.

Maps land in ~/.d1max/maps/<name>/ on this machine.
"""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import d1max_slam as slam   # noqa: E402

ORIN_HOST = "192.168.168.100"
ORIN_USER = "robot"
ORIN_PASS = "1"                       # documented default, docs/source/2.5
RK_HOST = "192.168.168.168"
REMOTE_DIR = "~/d1max_tools"
REMOTE_MAPS = "~/d1max_maps"

ROS_ENV = ("source /opt/ros/humble/setup.bash && "
           "export ROS_DOMAIN_ID=24 && "
           "export RMW_IMPLEMENTATION=rmw_zenoh_cpp")

TOPICS = ["/front_lidar", "/front_lidar/imu", "/tf", "/tf_static"]

HERE = os.path.dirname(os.path.abspath(__file__))
PUSH_FILES = ["d1max_proto.py", "d1max_client.py", "d1max_odom_bridge.py"]

# FAST-LIO preprocess.lidar_type values.
LIDAR_TYPE_NAMES = {1: "Livox", 2: "Velodyne-style (RoboSense)", 3: "Ouster"}


def lidar_type_for(fields: str) -> int:
    """Pick FAST-LIO's lidar_type from the PointCloud2 field names.

    The D1 Max ships RoboSense units (the robot's own install space carries
    rslidar_sdk / rslidar_msg). RoboSense clouds are laid out like Velodyne --
    ring + per-point time -- so type 2 is the right starting point, not the
    Livox default most FAST-LIO examples use.
    """
    f = fields.lower()
    if "t" in f.split() and "reflectivity" in f:
        return 3                      # Ouster: t, reflectivity, ambient
    if "ring" in f or "timestamp" in f or "time" in f:
        return 2                      # Velodyne / RoboSense
    if "line" in f or "offset_time" in f:
        return 1                      # Livox custom
    return 2


# ------------------------------------------------------------------ ssh glue
def have(cmd: str) -> bool:
    return subprocess.call(["which", cmd], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL) == 0


def ssh_prefix(host: str = ORIN_HOST, user: str = ORIN_USER,
               password: str | None = ORIN_PASS) -> list[str]:
    """ssh argv, using sshpass when a password is needed and keys aren't set up."""
    base = ["ssh", "-o", "StrictHostKeyChecking=no",
            "-o", "UserKnownHostsFile=/dev/null",
            "-o", "LogLevel=ERROR", "-o", "ConnectTimeout=8"]
    if password and not key_works(host, user):
        if not have("sshpass"):
            die("sshpass is not installed and no SSH key is set up.\n\n"
                "Pick one:\n"
                "    sudo apt install -y sshpass\n"
                "or set up a key once (nicer, no password after this):\n"
                f"    ssh-keygen -t ed25519 -N '' -f ~/.ssh/id_ed25519\n"
                f"    ssh-copy-id {user}@{host}          # password: {password}")
        base = ["sshpass", "-p", password] + base
    return base + [f"{user}@{host}"]


_key_cache: dict = {}


def require_ssh() -> None:
    if not have("ssh") or not have("scp"):
        die("the ssh client is not installed on this machine.\n\n"
            "    sudo apt install -y openssh-client sshpass")


def key_works(host: str, user: str) -> bool:
    """Does key-based (passwordless) SSH already work?"""
    k = (host, user)
    if k in _key_cache:
        return _key_cache[k]
    require_ssh()
    rc = subprocess.call(
        ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=no",
         "-o", "UserKnownHostsFile=/dev/null", "-o", "LogLevel=ERROR",
         "-o", "ConnectTimeout=6", f"{user}@{host}", "true"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    _key_cache[k] = (rc == 0)
    return _key_cache[k]


def run_remote(cmd: str, timeout: float = 60, check: bool = False,
               stream: bool = False) -> tuple[int, str]:
    argv = ssh_prefix() + [cmd]
    if stream:
        return subprocess.call(argv), ""
    try:
        r = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return 124, "timed out"
    out = (r.stdout + r.stderr).strip()
    if check and r.returncode != 0:
        die(f"remote command failed ({r.returncode}):\n  {cmd}\n{out}")
    return r.returncode, out


def ros_remote(cmd: str, **kw) -> tuple[int, str]:
    return run_remote(f"bash -lc {shlex.quote(ROS_ENV + ' && ' + cmd)}", **kw)


def scp_to(local: str, remote: str) -> None:
    base = ["scp", "-o", "StrictHostKeyChecking=no",
            "-o", "UserKnownHostsFile=/dev/null", "-o", "LogLevel=ERROR"]
    if not key_works(ORIN_HOST, ORIN_USER):
        base = ["sshpass", "-p", ORIN_PASS] + base
    subprocess.check_call(base + [local, f"{ORIN_USER}@{ORIN_HOST}:{remote}"])


def scp_from(remote: str, local: str, recursive: bool = False) -> int:
    base = ["scp", "-o", "StrictHostKeyChecking=no",
            "-o", "UserKnownHostsFile=/dev/null", "-o", "LogLevel=ERROR"]
    if recursive:
        base.append("-r")
    if not key_works(ORIN_HOST, ORIN_USER):
        base = ["sshpass", "-p", ORIN_PASS] + base
    return subprocess.call(base + [f"{ORIN_USER}@{ORIN_HOST}:{remote}", local])


def die(msg: str) -> None:
    print(f"\n{msg}\n", file=sys.stderr)
    raise SystemExit(1)


def say(msg: str = "") -> None:
    print(msg, flush=True)


# -------------------------------------------------------------------- doctor
def cmd_doctor(args) -> int:
    say("Checking the Orin NX …\n")

    say(f"  ssh {ORIN_USER}@{ORIN_HOST}")
    rc, out = run_remote("echo ok", timeout=15)
    if rc != 0:
        die(f"cannot reach the Orin NX at {ORIN_HOST}.\n\n"
            "  - Are you on the robot's network?\n"
            "  - On Wi-Fi you need the route to the wired subnet:\n"
            "      sudo ip route add 192.168.168.0/24 via 192.168.234.1\n"
            f"  - Try by hand: ssh {ORIN_USER}@{ORIN_HOST}    (password: {ORIN_PASS})\n\n"
            f"ssh said: {out}")
    say("    reachable ✓")
    say(f"    passwordless key: {'yes' if key_works(ORIN_HOST, ORIN_USER) else 'no (using sshpass)'}")

    rc, out = run_remote("ls /opt/ros 2>/dev/null || true")
    say(f"\n  ROS 2 on the Orin: {out or 'NONE'}")
    if "humble" not in out:
        say("    ! expected humble — mapping commands will not work")

    # The Orin runs its own Zenoh router. Starting a second one fails with
    # "Address already in use" -- that error means things are fine, not broken.
    rc, out = run_remote("ss -ltn 2>/dev/null | grep -c ':7447' || true")
    if out.strip() and out.strip() != "0":
        say("    zenoh router already running on :7447 ✓ (do NOT start another)")

    rc, out = ros_remote("ros2 topic list 2>/dev/null", timeout=40)
    topics = [t for t in out.splitlines() if t.startswith("/")]
    say(f"\n  Topics visible on the robot: {len(topics)}")
    for t in TOPICS + ["/rear_lidar", "/odom"]:
        say(f"    [{'x' if t in topics else ' '}] {t}")

    if "/front_lidar" not in topics:
        say("\n    ! /front_lidar missing. Check the LiDAR is powered and the")
        say("      driver is running:  robot-launch egg")
    else:
        # Which SLAM front end config we need depends entirely on the point
        # format, so read it rather than guess.
        rc, out = ros_remote(
            "timeout 12 ros2 topic echo /front_lidar --once --field fields "
            "2>/dev/null | grep name | awk '{print $2}' | tr '\\n' ' '",
            timeout=30)
        fields = out.strip()
        if fields:
            say(f"\n  /front_lidar point fields: {fields}")
            say(f"    -> lidar_type {lidar_type_for(fields)} "
                f"({LIDAR_TYPE_NAMES.get(lidar_type_for(fields), '?')})")

    rc, out = ros_remote("ros2 pkg list 2>/dev/null | grep -i -E 'fast_lio|point_lio' || true",
                         timeout=40)
    slam_pkgs = [p for p in out.splitlines() if p.strip()]
    say(f"\n  SLAM packages: {', '.join(slam_pkgs) or 'none installed'}")
    if not slam_pkgs:
        say("    run:  python3 tools/d1max_map.py install-slam")

    rc, out = run_remote("df -h ~ | tail -1 | awk '{print $4\" free of \"$2}'")
    say(f"\n  Disk on the Orin: {out}")

    ready = "/front_lidar" in topics
    say(f"\n{'READY TO RECORD' if ready else 'NOT READY'}")
    say("\nNext:  python3 tools/d1max_map.py record <name>")
    return 0 if ready else 2


# -------------------------------------------------------------------- record
def cmd_record(args) -> int:
    name = args.name
    say(f"Preparing to record '{name}' on the Orin NX …\n")

    rc, _ = run_remote("echo ok", timeout=15)
    if rc != 0:
        die("cannot reach the Orin NX — run `doctor` first")

    run_remote(f"mkdir -p {REMOTE_DIR} {REMOTE_MAPS}", check=True)

    # Push the odometry bridge so /odom exists in the bag. Without a motion
    # prior a LiDAR-inertial front end has a much harder time.
    if not args.no_odom:
        say("  copying the odometry bridge …")
        for f in PUSH_FILES:
            scp_to(os.path.join(HERE, f), f"{REMOTE_DIR}/")
        run_remote(f"pkill -f d1max_odom_bridge || true")
        bridge = (f"cd {REMOTE_DIR} && nohup python3 d1max_odom_bridge.py "
                  f"--host {RK_HOST} > /tmp/d1max_bridge.log 2>&1 &")
        ros_remote(bridge)
        time.sleep(4)
        rc, out = ros_remote("ros2 topic list 2>/dev/null | grep -c '^/odom$' || true",
                             timeout=30)
        if out.strip() == "1":
            say("  /odom publishing ✓")
        else:
            say("  ! /odom not up — check /tmp/d1max_bridge.log on the Orin.")
            say("    Recording anyway; SLAM can still run LiDAR+IMU only.")

    topics = list(TOPICS) + (["/odom"] if not args.no_odom else [])
    if args.rear:
        topics.append("/rear_lidar")
    bag = f"{REMOTE_MAPS}/{name}/bag"
    run_remote(f"rm -rf {bag}")

    say("\n" + "=" * 62)
    say("  RECORDING IS ABOUT TO START.")
    say("")
    say("  Drive the robot with the console in another window:")
    say("      python3 tools/d1max_console.py --host 192.168.234.1")
    say("")
    say("  Technique matters more than tuning:")
    say("    - Speed LOW, gentle inputs")
    say("    - Turn slowly; fast in-place spins break LiDAR-inertial SLAM")
    say("    - CLOSE THE LOOP — come back to where you started")
    say("    - Cover walls, corners and doorways")
    say("")
    say("  Press Ctrl-C here when you are done driving.")
    say("=" * 62 + "\n")
    input("  Press Enter to start recording… ")

    cmd = (f"mkdir -p {REMOTE_MAPS}/{name} && cd {REMOTE_MAPS}/{name} && "
           f"ros2 bag record -o bag {' '.join(topics)}")
    say(f"\n  recording {topics}\n  Ctrl-C to stop.\n")
    try:
        ros_remote(cmd, stream=True)
    except KeyboardInterrupt:
        pass

    run_remote("pkill -f 'ros2 bag record' || true")
    if not args.no_odom:
        run_remote("pkill -f d1max_odom_bridge || true")

    rc, out = run_remote(f"du -sh {bag} 2>/dev/null | cut -f1 || true")
    say(f"\n  bag recorded on the Orin: {out or 'unknown size'}")
    say(f"\nNext:")
    say(f"  python3 tools/d1max_map.py build {name}     # run SLAM over it")
    say(f"  python3 tools/d1max_map.py fetch {name}     # or pull the raw bag back")
    return 0


# --------------------------------------------------------------- install slam
FASTLIO_SETUP = r"""
set -e
sudo apt-get update -qq
sudo apt-get install -y -qq git python3-colcon-common-extensions \
    libpcl-dev libeigen3-dev ros-humble-pcl-conversions ros-humble-pcl-ros
mkdir -p ~/lio_ws/src
cd ~/lio_ws/src
[ -d FAST_LIO ] || git clone --recursive https://github.com/Ericsii/FAST_LIO.git
[ -d livox_ros_driver2 ] || git clone https://github.com/Livox-SDK/livox_ros_driver2.git || true
cd ~/lio_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fast_lio 2>&1 | tail -20
echo "FASTLIO_BUILD_DONE"
"""


def cmd_install_slam(args) -> int:
    say("Installing FAST-LIO2 on the Orin NX. This takes several minutes\n"
        "and needs the robot to have internet access.\n")
    rc, out = run_remote("ping -c1 -W3 github.com >/dev/null 2>&1 && echo net || echo nonet")
    if "nonet" in out:
        die("the Orin NX has no internet access, so it cannot fetch FAST-LIO2.\n\n"
            "Options:\n"
            "  - give the robot internet (it needs a route out for apt/git)\n"
            "  - or build FAST-LIO2 on a 22.04 machine and copy ~/lio_ws across\n"
            "  - or skip SLAM: `record` still works, and you can process the bag later")

    say("  building … (output tails below)\n")
    rc, _ = run_remote(f"bash -lc {shlex.quote(FASTLIO_SETUP)}", timeout=2400, stream=True)
    rc2, out = run_remote("ls ~/lio_ws/install/fast_lio 2>/dev/null && echo OK || echo MISSING")
    if "OK" not in out:
        die("the build did not produce ~/lio_ws/install/fast_lio.\n"
            "Scroll up for the compiler error, or build it by hand per "
            "docs/dev/07-slam-mapping-howto.md")
    say("\n  FAST-LIO2 installed ✓")
    say("\nNext:  python3 tools/d1max_map.py build <name>")
    return 0


# --------------------------------------------------------------------- build
CONFIG_TMPL = """common:
    lid_topic:  "/front_lidar"
    imu_topic:  "/front_lidar/imu"
    time_sync_en: false
preprocess:
    lidar_type: {lidar_type}          # {lidar_name}
    scan_line: 96                     # Airy is a 96-line unit
    blind: 0.5                        # ignore returns inside the robot's own body
    timestamp_unit: 3                 # RoboSense stamps in seconds
mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree: 360.0
    det_range: 100.0
    extrinsic_est_en: false
    extrinsic_T: [ 0.0, 0.0, 0.0 ]    # IMU lives inside the LiDAR housing
    extrinsic_R: [ 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 ]
publish:
    path_en: true
    scan_publish_en: true
    dense_publish_en: false
pcd_save:
    pcd_save_en: true
    interval: -1
"""



def cmd_build(args) -> int:
    name = args.name
    bag = f"{REMOTE_MAPS}/{name}/bag"

    rc, out = run_remote(f"test -d {bag} && echo yes || echo no")
    if "yes" not in out:
        die(f"no bag at {bag} on the Orin. Record one first:\n"
            f"    python3 tools/d1max_map.py record {name}")

    rc, out = run_remote("ls ~/lio_ws/install/fast_lio 2>/dev/null && echo OK || echo MISSING")
    if "OK" not in out:
        die("FAST-LIO2 is not installed on the Orin.\n"
            "    python3 tools/d1max_map.py install-slam")

    lt = args.lidar_type
    if not lt:
        rc, out = ros_remote(
            "timeout 12 ros2 topic echo /front_lidar --once --field fields "
            "2>/dev/null | grep name | awk '{print $2}' | tr '\\n' ' '", timeout=30)
        fields = out.strip()
        lt = lidar_type_for(fields) if fields else 2
        say(f"  /front_lidar fields: {fields or '(could not read — assuming RoboSense)'}")
    say(f"  lidar_type {lt} ({LIDAR_TYPE_NAMES.get(lt, '?')})")

    cfg = CONFIG_TMPL.format(lidar_type=lt,
                             lidar_name=LIDAR_TYPE_NAMES.get(lt, "?"))
    say("  writing config …")
    cfgpath = "~/lio_ws/src/FAST_LIO/config/d1max.yaml"
    run_remote(f"mkdir -p ~/lio_ws/src/FAST_LIO/config && "
               f"cat > {cfgpath} << 'D1MAXEOF'\n{cfg}\nD1MAXEOF", check=True)
    run_remote("rm -rf ~/lio_ws/src/FAST_LIO/PCD && mkdir -p ~/lio_ws/src/FAST_LIO/PCD")

    say("  starting FAST-LIO2 and replaying the bag …\n")
    play = (f"source ~/lio_ws/install/setup.bash && "
            f"(ros2 launch fast_lio mapping.launch.py config_file:=d1max.yaml "
            f" > /tmp/d1max_lio.log 2>&1 &) && sleep 6 && "
            f"ros2 bag play {bag} --rate {args.rate} && sleep 8 && "
            f"pkill -f fast_lio || true")
    ros_remote(play, timeout=7200, stream=True)

    rc, out = run_remote("ls -la ~/lio_ws/src/FAST_LIO/PCD/*.pcd 2>/dev/null || true")
    if ".pcd" not in out:
        die("SLAM produced no PCD. Check /tmp/d1max_lio.log on the Orin:\n"
            f"    ssh {ORIN_USER}@{ORIN_HOST} tail -50 /tmp/d1max_lio.log\n\n"
            "Common causes:\n"
            "  - lidar_type wrong. The D1 Max ships RoboSense (rslidar_sdk on the\n"
            "    robot), which is Velodyne-style: try --lidar-type 2, then 3, then 1.\n"
            "  - the bag has no /front_lidar/imu\n"
            "  - timestamp_unit mismatch (RoboSense stamps in seconds)")
    say(f"\n  cloud produced:\n{out}")
    say(f"\nNext:  python3 tools/d1max_map.py fetch {name}")
    return 0


# --------------------------------------------------------------------- fetch
def cmd_fetch(args) -> int:
    name = args.name
    dest = slam.map_path(name)
    os.makedirs(dest, exist_ok=True)

    rc, out = run_remote("ls ~/lio_ws/src/FAST_LIO/PCD/*.pcd 2>/dev/null | head -1 || true")
    pcd_remote = out.strip().splitlines()[0] if out.strip() else ""

    if pcd_remote:
        say(f"  fetching {pcd_remote} …")
        if scp_from(pcd_remote, os.path.join(dest, "map.pcd")) != 0:
            die("scp of the point cloud failed")
    else:
        say("  no PCD on the Orin (SLAM not run yet).")

    if args.bag:
        say("  fetching the bag (this can be large) …")
        scp_from(f"{REMOTE_MAPS}/{name}/bag", dest, recursive=True)

    pcd_local = os.path.join(dest, "map.pcd")
    if not os.path.exists(pcd_local):
        say(f"\nNothing to project yet. Saved to {dest}")
        return 0

    say("  projecting to a 2-D grid …")
    try:
        meta = slam.save_map(name, pcd_local, args.res, args.z_min, args.z_max,
                             args.min_hits)
    except Exception as exc:
        die(f"projection failed: {exc}")

    say(f"\nMap '{name}' saved to {dest}")
    for k in ("width", "height", "resolution", "ground_z", "cells_occupied",
              "points_total"):
        if k in meta:
            say(f"  {k:16} {meta[k]}")
    say("\nFiles: map.pcd  map.pgm  map.yaml  preview.png  meta.json")
    say("It now shows in the console's SLAM MAPS panel.")
    say(f"\nRetune the grid without re-mapping:")
    say(f"  python3 tools/d1max_slam.py grid --name {name} --z-min 0.2 --res 0.03")
    return 0


# ----------------------------------------------------------------------- CLI
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("doctor", help="check the Orin NX, ROS 2, topics and SLAM packages")

    r = sub.add_parser("record", help="record a mapping bag on the Orin while you drive")
    r.add_argument("name")
    r.add_argument("--rear", action="store_true", help="also record /rear_lidar")
    r.add_argument("--no-odom", action="store_true",
                   help="skip the odometry bridge (LiDAR + IMU only)")

    sub.add_parser("install-slam", help="build FAST-LIO2 on the Orin NX (one-time)")

    b = sub.add_parser("build", help="run SLAM over a recorded bag on the Orin")
    b.add_argument("name")
    b.add_argument("--rate", type=float, default=1.0, help="bag playback rate")
    b.add_argument("--lidar-type", type=int, choices=[1, 2, 3], default=None,
                   help="FAST-LIO lidar_type; detected from the point fields if omitted")

    f = sub.add_parser("fetch", help="pull the cloud back and project it to 2-D")
    f.add_argument("name")
    f.add_argument("--bag", action="store_true", help="also copy the raw bag")
    f.add_argument("--res", type=float, default=slam.DEFAULT_RES)
    f.add_argument("--z-min", type=float, default=slam.DEFAULT_Z_MIN)
    f.add_argument("--z-max", type=float, default=slam.DEFAULT_Z_MAX)
    f.add_argument("--min-hits", type=int, default=slam.DEFAULT_HITS)

    args = ap.parse_args()
    require_ssh()
    return {"doctor": cmd_doctor, "record": cmd_record,
            "install-slam": cmd_install_slam, "build": cmd_build,
            "fetch": cmd_fetch}[args.cmd](args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\ninterrupted")
        raise SystemExit(130)

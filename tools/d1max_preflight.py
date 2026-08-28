#!/usr/bin/env python3
"""End-to-end readiness check for mapping and map-frame missions.

d1max_slam.check_env() inspects THIS laptop -- local /opt/ros, local rclpy,
local topic list. That is the wrong machine: mapping and localisation run on
the Orin NX over SSH, and the laptop deliberately needs no ROS 2 at all. This
walks the actual dependency chain instead, on the machine that owns each link,
and names the fix for whichever one is broken.

    python3 tools/d1max_preflight.py
"""

from __future__ import annotations

import shlex
import socket
import subprocess
import sys

import d1max_map as M

OK, WARN, FAIL, SKIP = "ok", "warn", "fail", "skip"


class Check:
    def __init__(self, key, label, needs=None):
        self.key, self.label, self.needs = key, label, needs or []


def _row(key, label, state, detail="", fix=""):
    return {"key": key, "label": label, "state": state,
            "detail": detail, "fix": fix}


def _remote(cmd, timeout=25):
    try:
        return M.run_remote(cmd, timeout=timeout)
    except SystemExit as exc:
        return 255, str(exc)
    except Exception as exc:                      # noqa: BLE001
        return 255, str(exc)


def _ros(cmd, timeout=40):
    try:
        return M.ros_remote(cmd, timeout=timeout)
    except SystemExit as exc:
        return 255, str(exc)
    except Exception as exc:                      # noqa: BLE001
        return 255, str(exc)


def _ping(dest: str, timeout: int = 2) -> bool:
    """Ground truth for reachability.

    A UDP connect() is NOT enough: it succeeds via the default route for any
    address, so it reports success on a completely unrelated network. Only an
    actual reply proves the robot is there.
    """
    return subprocess.call(["ping", "-c1", "-W", str(timeout), dest],
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL) == 0


def _route_to(dest: str) -> str:
    """`ip route get` summary, for explaining *why* something is unreachable."""
    try:
        r = subprocess.run(["ip", "route", "get", dest],
                           capture_output=True, text=True, timeout=5)
        return r.stdout.strip().splitlines()[0] if r.stdout.strip() else ""
    except Exception:                              # noqa: BLE001
        return ""


def _has_specific_route(dest: str, via: str) -> bool:
    return f"via {via}" in _route_to(dest)


def run(deep: bool = True) -> dict:
    rows = []

    # ---- 1. link to the control board (this is the SDK path) -------------
    AP = "192.168.234.1"
    if _ping(AP):
        rows.append(_row("ap", "Robot hotspot reachable", OK,
                         f"{AP} replies  ({_route_to(AP)})"))
    else:
        rows.append(_row("ap", "Robot hotspot reachable", FAIL,
                         f"no reply from {AP}  ({_route_to(AP) or 'no route'})",
                         "Join the robot's Wi-Fi: XG2WIFI_* (password 12345678)"))
        for k, l in (("route", "Route to 192.168.168.0/24"),
                     ("ping", "Orin NX responds to ping"),
                     ("ssh", "SSH to the Orin NX"),
                     ("ros", "ROS 2 Humble on the Orin NX"),
                     ("lidar", "RoboSense Airy topics present"),
                     ("lidar_data", "Airy is publishing points"),
                     ("fastlio", "FAST-LIO2 built on the Orin NX"),
                     ("numpy", "numpy on the Orin NX (localiser)"),
                     ("maps", "Maps on the Orin NX")):
            rows.append(_row(k, l, SKIP))
        return _finish(rows)

    # ---- 2. route to the Orin NX subnet ---------------------------------
    if _has_specific_route(M.ORIN_HOST, AP):
        rows.append(_row("route", "Route to 192.168.168.0/24", OK,
                         _route_to(M.ORIN_HOST)))
    else:
        rows.append(_row("route", "Route to 192.168.168.0/24", FAIL,
                         f"not routed via {AP}  ({_route_to(M.ORIN_HOST) or 'no route'})",
                         "sudo ip route add 192.168.168.0/24 via 192.168.234.1"))
        for k, l in (("ping", "Orin NX responds to ping"),
                     ("ssh", "SSH to the Orin NX"),
                     ("ros", "ROS 2 Humble on the Orin NX"),
                     ("lidar", "RoboSense Airy topics present"),
                     ("lidar_data", "Airy is publishing points"),
                     ("fastlio", "FAST-LIO2 built on the Orin NX"),
                     ("numpy", "numpy on the Orin NX (localiser)"),
                     ("maps", "Maps on the Orin NX")):
            rows.append(_row(k, l, SKIP))
        return _finish(rows)

    # ---- 3. ping + SSH ---------------------------------------------------
    if _ping(M.ORIN_HOST):
        rows.append(_row("ping", "Orin NX responds to ping", OK, M.ORIN_HOST))
    else:
        rows.append(_row("ping", "Orin NX responds to ping", FAIL,
                         f"no reply from {M.ORIN_HOST}",
                         "Route exists but the board is not answering. Is it booted?"))
        return _finish(rows)

    rc, out = _remote("echo ok")
    if rc == 0 and "ok" in out:
        rows.append(_row("ssh", "SSH to the Orin NX", OK,
                         f"{M.ORIN_USER}@{M.ORIN_HOST}"))
    else:
        rows.append(_row("ssh", "SSH to the Orin NX", FAIL, out[:160],
                         f"sudo apt install -y sshpass, or set up a key:\n"
                         f"ssh-copy-id {M.ORIN_USER}@{M.ORIN_HOST}   "
                         f"(password: {M.ORIN_PASS})"))
        return _finish(rows)

    # ---- 4. ROS 2 on the NX ---------------------------------------------
    rc, out = _ros("ros2 --help >/dev/null 2>&1 && echo yes || echo no")
    if out.strip().endswith("yes"):
        rows.append(_row("ros", "ROS 2 Humble on the Orin NX", OK))
    else:
        rows.append(_row("ros", "ROS 2 Humble on the Orin NX", FAIL, out[:160],
                         "The robot image should ship Humble. Check "
                         "/opt/ros/humble on the Orin."))
        return _finish(rows)

    # ---- 5. lidar topics -------------------------------------------------
    rc, out = _ros("ros2 topic list 2>/dev/null | tr '\\n' ' '")
    topics = out.split()
    have_front = "/front_lidar" in topics
    have_rear = "/rear_lidar" in topics
    if have_front:
        rows.append(_row("lidar", "RoboSense Airy topics present", OK,
                         "/front_lidar" + (" + /rear_lidar" if have_rear else
                                           "  (rear not advertised)")))
    else:
        rows.append(_row("lidar", "RoboSense Airy topics present", FAIL,
                         f"{len(topics)} topics, no /front_lidar",
                         "Lidar driver not running on the robot, or the topic is "
                         "named differently. Seen: " + " ".join(topics[:12])))

    # ---- 6. is the lidar actually publishing? ---------------------------
    if have_front and deep:
        rc, out = _ros("timeout 8 ros2 topic echo /front_lidar --no-arr --once "
                       "2>/dev/null | grep -E 'frame_id|width' | tr '\\n' ' '",
                       timeout=30)
        if "width" in out:
            rows.append(_row("lidar_data", "Airy is publishing points", OK, out.strip()[:120]))
        else:
            rows.append(_row("lidar_data", "Airy is publishing points", FAIL,
                             "topic advertised but nothing arrived in 8 s",
                             "Check the lidar is powered and the driver is healthy."))
    else:
        rows.append(_row("lidar_data", "Airy is publishing points", SKIP))

    # ---- 7. FAST-LIO2, the usual blocker --------------------------------
    rc, out = _remote("ls ~/lio_ws/install/fast_lio >/dev/null 2>&1 && echo yes || echo no")
    if out.strip().endswith("yes"):
        rows.append(_row("fastlio", "FAST-LIO2 built on the Orin NX", OK))
    else:
        rc2, net = _remote("ping -c1 -W3 github.com >/dev/null 2>&1 && echo net || echo nonet")
        has_net = net.strip().endswith("net")
        rows.append(_row(
            "fastlio", "FAST-LIO2 built on the Orin NX", FAIL,
            "not installed" + ("" if has_net else "; robot also has NO internet"),
            "python3 tools/d1max_map.py install-slam" if has_net else
            "The robot has no internet, so install-slam cannot fetch it. Give the "
            "robot a route out, or build lio_ws on another 22.04 machine and copy "
            "it to ~/lio_ws on the Orin. Recording still works without this."))

    # ---- 8. numpy, needed by the localizer ------------------------------
    rc, out = _remote("python3 -c 'import numpy;print(numpy.__version__)' 2>&1 | tail -1")
    if rc == 0 and out and out[0].isdigit():
        rows.append(_row("numpy", "numpy on the Orin NX (localiser)", OK, out.strip()))
    else:
        rows.append(_row("numpy", "numpy on the Orin NX (localiser)", FAIL,
                         out[:120], "ssh onto the Orin and: sudo apt install -y python3-numpy"))

    # ---- 9. maps present -------------------------------------------------
    rc, out = _remote(f"ls {M.REMOTE_MAPS} 2>/dev/null | tr '\\n' ' '")
    remote_maps = out.split() if rc == 0 else []
    rows.append(_row("maps", "Maps on the Orin NX", OK if remote_maps else WARN,
                     " ".join(remote_maps) if remote_maps else "none yet",
                     "" if remote_maps else
                     "Expected until you record and build your first map."))
    return _finish(rows)


def _finish(rows):
    order = {FAIL: 0, WARN: 1, SKIP: 2, OK: 3}
    worst = min((order[r["state"]] for r in rows), default=3)
    blocking = [r for r in rows if r["state"] == FAIL]
    # Recording only needs the chain up to and including a publishing lidar.
    rec_keys = {"ap", "route", "ping", "ssh", "ros", "lidar"}
    can_record = not any(r["state"] == FAIL and r["key"] in rec_keys for r in rows)
    can_map = can_record and not any(
        r["state"] == FAIL and r["key"] == "fastlio" for r in rows)
    can_localize = can_map and not any(
        r["state"] == FAIL and r["key"] == "numpy" for r in rows)
    return {
        "rows": rows,
        "ready": worst >= 2,
        "can_record": can_record,
        "can_build_map": can_map,
        "can_localize": can_localize,
        "blocking": [r["label"] for r in blocking],
    }


def main() -> int:
    r = run()
    sym = {OK: "  ok  ", WARN: " warn ", FAIL: " FAIL ", SKIP: " --   "}
    print()
    for row in r["rows"]:
        print(f"[{sym[row['state']]}] {row['label']}")
        if row["detail"]:
            print(f"           {row['detail']}")
        if row["state"] in (FAIL, WARN) and row["fix"]:
            for line in row["fix"].splitlines():
                print(f"           -> {line}")
    print()
    print(f"  record a bag : {'YES' if r['can_record'] else 'no'}")
    print(f"  build a map  : {'YES' if r['can_build_map'] else 'no'}")
    print(f"  localise     : {'YES' if r['can_localize'] else 'no'}")
    print()
    return 0 if r["can_record"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

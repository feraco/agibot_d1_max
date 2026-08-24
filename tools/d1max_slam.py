#!/usr/bin/env python3
"""SLAM mapping helper for the D1 Max: check, record, save, convert, list.

The robot ships LiDAR and IMU on ROS 2 from the Orin NX but no odometry, no
map and no SLAM node. This tool wraps the workflow around whatever LiDAR-
inertial front-end you install (FAST-LIO2 or Point-LIO are the recommended
pair -- see docs/dev/07-slam-mapping-howto.md):

    python3 d1max_slam.py check              # is the environment usable?
    python3 d1max_slam.py record --name lab  # ros2 bag the mapping topics
    python3 d1max_slam.py save --name lab --pcd /path/scans.pcd
    python3 d1max_slam.py grid  --name lab   # (re)project the PCD to a 2-D map
    python3 d1max_slam.py list

`grid` is pure Python -- no ROS, no PCL, no numpy. It turns a PCD from any
SLAM package into a nav2-style occupancy grid (map.pgm + map.yaml) plus a PNG
preview, which is what the console UI and any 2-D planner actually need.

Maps live in ~/.d1max/maps/<name>/.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import struct
import subprocess
import sys
import time
import zlib

MAP_DIR = os.path.expanduser("~/.d1max/maps")

# Topics a LiDAR-inertial map needs. /odom comes from d1max_odom_bridge.py.
MAPPING_TOPICS = [
    "/front_lidar",
    "/front_lidar/imu",
    "/odom",
    "/tf",
    "/tf_static",
]
OPTIONAL_TOPICS = ["/rear_lidar", "/imu_driver/imu_central", "/rtk_pvh"]

# Height band kept when flattening the cloud to 2-D, relative to the estimated
# ground plane. Clears the floor and the overhangs the robot walks under.
DEFAULT_Z_MIN = 0.15
DEFAULT_Z_MAX = 1.20
DEFAULT_RES = 0.05
DEFAULT_HITS = 2


# ============================================================== environment
def sh(cmd: list[str], timeout: float = 6.0) -> tuple[int, str]:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr).strip()
    except FileNotFoundError:
        return 127, "not found"
    except subprocess.TimeoutExpired:
        return 124, "timed out"


def os_release() -> dict:
    """Parse /etc/os-release. Empty dict on anything that isn't Linux."""
    info = {}
    try:
        with open("/etc/os-release") as fh:
            for line in fh:
                k, _, v = line.strip().partition("=")
                if k:
                    info[k] = v.strip('"')
    except OSError:
        pass
    return info


# ROS 2 distro that ships with each Ubuntu release.
DISTRO_FOR_UBUNTU = {"22.04": "humble", "24.04": "jazzy", "20.04": "foxy"}
ROBOT_DISTRO = "humble"


def check_env(verbose: bool = True) -> dict:
    """Diagnose the ROS 2 side without assuming any of it is present."""
    out: dict = {}

    rel = os_release()
    out["os_name"] = rel.get("PRETTY_NAME", "unknown")
    out["ubuntu_version"] = rel.get("VERSION_ID", "")
    out["expected_distro"] = DISTRO_FOR_UBUNTU.get(out["ubuntu_version"], "")
    out["ros_installed"] = sorted(
        d for d in (os.listdir("/opt/ros") if os.path.isdir("/opt/ros") else [])
        if os.path.isdir(os.path.join("/opt/ros", d)))

    # conda's python shadows the system one and ROS 2 packages then fail to
    # import -- an extremely common and confusing failure.
    out["conda"] = os.environ.get("CONDA_DEFAULT_ENV") or ""
    out["conda_python"] = "conda" in sys.executable or "anaconda" in sys.executable

    out["ros_distro"] = os.environ.get("ROS_DISTRO") or ""
    out["domain_id"] = os.environ.get("ROS_DOMAIN_ID") or ""
    out["rmw"] = os.environ.get("RMW_IMPLEMENTATION") or ""

    rc, _ = sh(["ros2", "--help"], timeout=10)
    out["ros2_cli"] = rc == 0

    try:
        import rclpy  # noqa: F401
        out["rclpy"] = True
    except ImportError:
        out["rclpy"] = False

    topics: list[str] = []
    if out["ros2_cli"]:
        rc, txt = sh(["ros2", "topic", "list"], timeout=12)
        if rc == 0:
            topics = [t.strip() for t in txt.splitlines() if t.strip().startswith("/")]
    out["topics"] = topics
    out["have"] = {t: (t in topics) for t in MAPPING_TOPICS}
    out["optional"] = {t: (t in topics) for t in OPTIONAL_TOPICS}
    out["lidar_ok"] = "/front_lidar" in topics
    out["odom_ok"] = "/odom" in topics

    issues = []
    if not out["ros_installed"]:
        want = out["expected_distro"] or ROBOT_DISTRO
        issues.append(f"ROS 2 is not installed (/opt/ros is empty or missing). "
                      f"Run: python3 tools/d1max_slam.py install")
        if out["ubuntu_version"] and want != ROBOT_DISTRO:
            issues.append(
                f"Ubuntu {out['ubuntu_version']} ships ROS 2 {want}, but the robot "
                f"runs {ROBOT_DISTRO}. Mixing distros over Zenoh is not reliable — "
                f"prefer Ubuntu 22.04, a container, or run the bridge on the Orin NX.")
    elif not out["ros2_cli"]:
        d = out["ros_installed"][0]
        issues.append(f"ros2 CLI not on PATH — source /opt/ros/{d}/setup.bash")

    if out["conda_python"] or out["conda"]:
        issues.append(f"conda env '{out['conda'] or 'base'}' is active; its Python "
                      "shadows the system one and ROS 2 imports will fail. "
                      "Run: conda deactivate")
    if out["domain_id"] != "24":
        issues.append(f"ROS_DOMAIN_ID is {out['domain_id'] or 'unset'}, robot uses 24")
    if out["rmw"] != "rmw_zenoh_cpp":
        issues.append(f"RMW_IMPLEMENTATION is {out['rmw'] or 'unset'}, robot uses rmw_zenoh_cpp")
    if out["ros2_cli"] and not out["lidar_ok"]:
        issues.append("/front_lidar not visible — is the Zenoh router running and pointed at 192.168.168.100:7447?")
    if out["ros2_cli"] and not out["odom_ok"]:
        issues.append("/odom not visible — start d1max_odom_bridge.py")
    out["issues"] = issues
    out["ready"] = out["ros2_cli"] and out["lidar_ok"] and out["odom_ok"]

    if verbose:
        print("System")
        print(f"  OS                {out['os_name']}")
        print(f"  ROS 2 installed   {', '.join(out['ros_installed']) or 'NONE'}"
              f"   (robot uses {ROBOT_DISTRO})")
        if out["conda"] or out["conda_python"]:
            print(f"  conda env         {out['conda'] or '(unnamed)'}  <-- deactivate it")
        print("\nROS 2 environment")
        print(f"  distro            {out['ros_distro'] or '—'}")
        print(f"  ROS_DOMAIN_ID     {out['domain_id'] or '—'}   (robot uses 24)")
        print(f"  RMW               {out['rmw'] or '—'}   (robot uses rmw_zenoh_cpp)")
        print(f"  ros2 CLI          {'yes' if out['ros2_cli'] else 'NO'}")
        print(f"  rclpy importable  {'yes' if out['rclpy'] else 'NO'}")
        print(f"\nTopics visible: {len(topics)}")
        for t in MAPPING_TOPICS:
            print(f"  [{'x' if out['have'][t] else ' '}] {t}")
        for t in OPTIONAL_TOPICS:
            print(f"  [{'x' if out['optional'][t] else ' '}] {t}   (optional)")
        if issues:
            print("\nIssues:")
            for i in issues:
                print(f"  ! {i}")
        print(f"\n{'READY TO MAP' if out['ready'] else 'NOT READY'}")
    return out


# ==================================================================== PCD io
def read_pcd(path: str, max_points: int = 4_000_000):
    """Minimal PCD reader -> list of (x, y, z). Handles ascii and binary."""
    with open(path, "rb") as fh:
        fields, size, ftype, count = [], [], [], []
        npoints, data_kind = 0, "ascii"
        while True:
            raw = fh.readline()
            if not raw:
                raise ValueError("PCD header ended unexpectedly")
            line = raw.decode("ascii", "replace").strip()
            if not line or line.startswith("#"):
                continue
            key, _, rest = line.partition(" ")
            key = key.upper()
            if key == "FIELDS":
                fields = rest.split()
            elif key == "SIZE":
                size = [int(v) for v in rest.split()]
            elif key == "TYPE":
                ftype = rest.split()
            elif key == "COUNT":
                count = [int(v) for v in rest.split()]
            elif key == "POINTS":
                npoints = int(rest)
            elif key == "WIDTH" and not npoints:
                npoints = int(rest.split()[0])
            elif key == "DATA":
                data_kind = rest.strip().lower()
                break

        if not fields:
            raise ValueError("PCD has no FIELDS")
        if not count:
            count = [1] * len(fields)
        try:
            ix, iy, iz = fields.index("x"), fields.index("y"), fields.index("z")
        except ValueError:
            raise ValueError("PCD has no x/y/z fields")

        pts = []
        if data_kind == "ascii":
            for line in fh:
                parts = line.split()
                if len(parts) <= iz:
                    continue
                try:
                    pts.append((float(parts[ix]), float(parts[iy]), float(parts[iz])))
                except ValueError:
                    continue
                if len(pts) >= max_points:
                    break
            return pts

        if data_kind == "binary_compressed":
            raise ValueError("binary_compressed PCD is not supported — "
                             "re-save as binary or ascii")

        # binary: fixed-width records
        fmt_map = {("F", 4): "f", ("F", 8): "d",
                   ("U", 1): "B", ("U", 2): "H", ("U", 4): "I",
                   ("I", 1): "b", ("I", 2): "h", ("I", 4): "i"}
        fmt = "<"
        for t, s, c in zip(ftype, size, count):
            ch = fmt_map.get((t.upper(), s))
            if ch is None:
                raise ValueError(f"unsupported PCD field type {t}{s}")
            fmt += ch * c
        rec = struct.calcsize(fmt)
        # index of x within the flattened tuple
        flat = []
        for i, c in enumerate(count):
            flat.extend([i] * c)
        gx = flat.index(ix); gy = flat.index(iy); gz = flat.index(iz)

        blob = fh.read()
        n = min(npoints or len(blob) // rec, len(blob) // rec, max_points)
        unpack = struct.Struct(fmt).unpack_from
        for i in range(n):
            try:
                v = unpack(blob, i * rec)
            except struct.error:
                break
            x, y, z = v[gx], v[gy], v[gz]
            if x == x and y == y and z == z:      # drop NaN
                pts.append((float(x), float(y), float(z)))
        return pts


# ============================================================ 2-D projection
def ground_z(pts, sample: int = 200_000) -> float:
    """Estimate the floor as a low percentile of z."""
    zs = sorted(pt[2] for pt in pts[:sample])
    if not zs:
        return 0.0
    return zs[int(len(zs) * 0.02)]


def project_grid(pts, res=DEFAULT_RES, z_min=DEFAULT_Z_MIN, z_max=DEFAULT_Z_MAX,
                 min_hits=DEFAULT_HITS):
    """Flatten a cloud to an occupancy grid. Returns (cells, meta)."""
    if not pts:
        raise ValueError("no points")
    g = ground_z(pts)
    lo, hi = g + z_min, g + z_max

    counts: dict[tuple[int, int], int] = {}
    minx = miny = 1e18
    maxx = maxy = -1e18
    for x, y, z in pts:
        if z < lo or z > hi:
            continue
        cx, cy = int(math.floor(x / res)), int(math.floor(y / res))
        counts[(cx, cy)] = counts.get((cx, cy), 0) + 1
        if x < minx: minx = x
        if x > maxx: maxx = x
        if y < miny: miny = y
        if y > maxy: maxy = y

    occupied = {k for k, v in counts.items() if v >= min_hits}
    if not occupied:
        raise ValueError("no cells above the hit threshold — try --min-hits 1 "
                         "or a wider z band")

    cxs = [k[0] for k in occupied]; cys = [k[1] for k in occupied]
    x0, x1 = min(cxs), max(cxs)
    y0, y1 = min(cys), max(cys)
    pad = 4
    x0 -= pad; y0 -= pad; x1 += pad; y1 += pad
    w, h = x1 - x0 + 1, y1 - y0 + 1

    meta = {
        "resolution": res, "width": w, "height": h,
        "origin": [x0 * res, y0 * res, 0.0],
        "ground_z": round(g, 3), "z_band": [z_min, z_max],
        "min_hits": min_hits,
        "points_total": len(pts), "cells_occupied": len(occupied),
        "bounds": [round(minx, 2), round(miny, 2), round(maxx, 2), round(maxy, 2)],
    }
    return (occupied, x0, y0, w, h), meta


def write_pgm(path, grid) -> None:
    """nav2/map_server-compatible PGM: 0 occupied, 254 free."""
    occupied, x0, y0, w, h = grid
    rows = bytearray()
    for r in range(h):
        wy = y0 + (h - 1 - r)          # PGM row 0 is the top = max y
        rows.extend(bytes(0 if (x0 + c, wy) in occupied else 254 for c in range(w)))
    with open(path, "wb") as fh:
        fh.write(f"P5\n{w} {h}\n255\n".encode())
        fh.write(rows)


def write_yaml(path, pgm_name, meta) -> None:
    with open(path, "w") as fh:
        fh.write(
            f"image: {pgm_name}\n"
            f"resolution: {meta['resolution']}\n"
            f"origin: [{meta['origin'][0]:.4f}, {meta['origin'][1]:.4f}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")


def write_png(path, grid, bg=(13, 17, 22), fg=(63, 182, 193)) -> None:
    """Tiny stdlib PNG writer, for the console's map preview."""
    occupied, x0, y0, w, h = grid
    raw = bytearray()
    for r in range(h):
        wy = y0 + (h - 1 - r)
        raw.append(0)                                   # filter type: none
        for c in range(w):
            raw.extend(fg if (x0 + c, wy) in occupied else bg)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
           + chunk(b"IEND", b""))
    with open(path, "wb") as fh:
        fh.write(png)


# ================================================================== map store
def map_path(name: str) -> str:
    safe = "".join(ch for ch in name if ch.isalnum() or ch in "-_ ").strip() or "map"
    return os.path.join(MAP_DIR, safe.replace(" ", "_"))


def list_maps() -> list[dict]:
    os.makedirs(MAP_DIR, exist_ok=True)
    out = []
    for d in sorted(os.listdir(MAP_DIR)):
        full = os.path.join(MAP_DIR, d)
        meta_f = os.path.join(full, "meta.json")
        if not os.path.isdir(full):
            continue
        meta = {}
        if os.path.exists(meta_f):
            try:
                meta = json.load(open(meta_f))
            except (OSError, ValueError):
                meta = {}
        meta.update({
            "name": d, "path": full,
            "has_pcd": os.path.exists(os.path.join(full, "map.pcd")),
            "has_grid": os.path.exists(os.path.join(full, "map.pgm")),
            "has_png": os.path.exists(os.path.join(full, "preview.png")),
        })
        out.append(meta)
    return sorted(out, key=lambda m: m.get("created_at", 0), reverse=True)


def save_map(name: str, pcd: str | None, res: float, z_min: float, z_max: float,
             min_hits: int) -> dict:
    dest = map_path(name)
    os.makedirs(dest, exist_ok=True)
    meta = {"name": name, "created_at": time.time()}

    if pcd:
        if not os.path.exists(pcd):
            raise FileNotFoundError(pcd)
        target = os.path.join(dest, "map.pcd")
        if os.path.abspath(pcd) != os.path.abspath(target):
            shutil.copy2(pcd, target)
        meta["pcd_bytes"] = os.path.getsize(target)

    target = os.path.join(dest, "map.pcd")
    if os.path.exists(target):
        pts = read_pcd(target)
        grid, gmeta = project_grid(pts, res, z_min, z_max, min_hits)
        write_pgm(os.path.join(dest, "map.pgm"), grid)
        write_yaml(os.path.join(dest, "map.yaml"), "map.pgm", gmeta)
        write_png(os.path.join(dest, "preview.png"), grid)
        meta.update(gmeta)

    with open(os.path.join(dest, "meta.json"), "w") as fh:
        json.dump(meta, fh, indent=2)
    return meta


# ======================================================================= CLI
def cmd_install(args) -> int:
    """Print the exact install steps for this machine. Prints, never runs --
    installing a distro is your call, not a side effect of a diagnostic."""
    env = check_env(verbose=False)
    ver = env["ubuntu_version"]
    want = env["expected_distro"]

    print(f"Detected: {env['os_name']}")
    print(f"Robot runs ROS 2 {ROBOT_DISTRO} (Ubuntu 22.04).\n")

    if env["conda"] or env["conda_python"]:
        print("FIRST — leave conda. Its Python breaks ROS 2 imports:\n")
        print("    conda deactivate")
        print("    # to stop it auto-activating in new shells:")
        print("    conda config --set auto_activate_base false\n")

    if ver and ver != "22.04":
        print(f"! Ubuntu {ver} cannot install {ROBOT_DISTRO} from apt.")
        print(f"  Its native distro is {want or 'unknown'}, and mixing distros")
        print("  across Zenoh is not reliable. Pick one:\n")
        print("  a) Run mapping on the Orin NX itself (it already has Humble):")
        print("       ssh robot@192.168.168.100      # password: 1")
        print("     then copy tools/ across and run the bridge there.\n")
        print("  b) Use a container on this machine:")
        print("       sudo apt install -y docker.io")
        print("       sudo docker run -it --net=host --rm ros:humble bash\n")
        print("  c) Install Ubuntu 22.04 (dual boot or VM).\n")
        print("  You do NOT need any of this for missions — those work now.")
        return 0

    print("Install ROS 2 Humble:\n")
    print("    sudo apt update && sudo apt install -y software-properties-common curl")
    print("    sudo add-apt-repository universe -y")
    print("    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \\")
    print("         -o /usr/share/keyrings/ros-archive-keyring.gpg")
    print('    echo "deb [arch=$(dpkg --print-architecture) '
          'signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] '
          'http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \\')
    print("         | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null")
    print("    sudo apt update")
    print("    sudo apt install -y ros-humble-desktop ros-humble-rmw-zenoh-cpp \\")
    print("         ros-dev-tools python3-colcon-common-extensions\n")
    print("Then, in every terminal that talks to the robot:\n")
    print("    conda deactivate")
    print("    source /opt/ros/humble/setup.bash")
    print("    export ROS_DOMAIN_ID=24")
    print("    export RMW_IMPLEMENTATION=rmw_zenoh_cpp\n")
    print("Point Zenoh at the robot — edit this file's connect/endpoints:")
    print("    /opt/ros/humble/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5")
    print('    → "tcp/192.168.168.100:7447"\n')
    print("Verify:  python3 tools/d1max_slam.py check")
    return 0


def cmd_record(args) -> int:
    env = check_env(verbose=False)
    if not env["ros2_cli"]:
        print("ros2 CLI not available — source /opt/ros/humble/setup.bash", file=sys.stderr)
        return 1
    dest = map_path(args.name)
    os.makedirs(dest, exist_ok=True)
    bag = os.path.join(dest, "bag")
    topics = [t for t in MAPPING_TOPICS if env["have"].get(t)]
    if args.rear:
        topics.append("/rear_lidar")
    if not topics:
        print("none of the mapping topics are visible; run `check` first", file=sys.stderr)
        return 1
    print(f"recording {topics} -> {bag}\nCtrl-C to stop.")
    return subprocess.call(["ros2", "bag", "record", "-o", bag] + topics)


def cmd_save(args) -> int:
    try:
        meta = save_map(args.name, args.pcd, args.res, args.z_min, args.z_max,
                        args.min_hits)
    except Exception as exc:
        print(f"save failed: {exc}", file=sys.stderr)
        return 1
    print(f"saved map '{args.name}' -> {map_path(args.name)}")
    for k in ("width", "height", "resolution", "cells_occupied", "points_total",
              "ground_z"):
        if k in meta:
            print(f"  {k:16} {meta[k]}")
    return 0


def cmd_grid(args) -> int:
    dest = map_path(args.name)
    pcd = os.path.join(dest, "map.pcd")
    if not os.path.exists(pcd):
        print(f"no map.pcd in {dest}", file=sys.stderr)
        return 1
    return cmd_save(args)


def cmd_list(args) -> int:
    maps = list_maps()
    if not maps:
        print(f"no maps in {MAP_DIR}")
        return 0
    for m in maps:
        when = time.strftime("%Y-%m-%d %H:%M", time.localtime(m.get("created_at", 0)))
        size = f"{m.get('width','?')}x{m.get('height','?')}"
        print(f"  {m['name']:24} {when}  {size:>12}  "
              f"pcd={'y' if m['has_pcd'] else 'n'} grid={'y' if m['has_grid'] else 'n'}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("check", help="diagnose the ROS 2 mapping environment")
    sub.add_parser("install", help="print the ROS 2 install steps for this machine")

    r = sub.add_parser("record", help="ros2 bag the topics needed for mapping")
    r.add_argument("--name", required=True)
    r.add_argument("--rear", action="store_true", help="also record /rear_lidar")

    def grid_args(q):
        q.add_argument("--res", type=float, default=DEFAULT_RES)
        q.add_argument("--z-min", type=float, default=DEFAULT_Z_MIN)
        q.add_argument("--z-max", type=float, default=DEFAULT_Z_MAX)
        q.add_argument("--min-hits", type=int, default=DEFAULT_HITS)

    s = sub.add_parser("save", help="store a PCD as a named map and project it to 2-D")
    s.add_argument("--name", required=True)
    s.add_argument("--pcd", help="PCD produced by your SLAM node")
    grid_args(s)

    g = sub.add_parser("grid", help="re-project an already-saved map's PCD")
    g.add_argument("--name", required=True)
    g.add_argument("--pcd", default=None)
    grid_args(g)

    sub.add_parser("list", help="list saved maps")

    args = ap.parse_args()
    if args.cmd == "check":
        return 0 if check_env()["ready"] else 2
    return {"install": cmd_install, "record": cmd_record, "save": cmd_save,
            "grid": cmd_grid, "list": cmd_list}[args.cmd](args)


if __name__ == "__main__":
    raise SystemExit(main())

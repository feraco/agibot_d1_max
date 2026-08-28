#!/usr/bin/env python3
"""Map-frame localisation, so waypoints stop drifting with odometry.

Runs ON THE ORIN NX, which already has ROS 2 Humble and both RoboSense Airy
units. Your laptop stays ROS-free: this serves the map-frame pose over plain
HTTP, and the console polls it.

    # on the NX (pushed there by d1max_map.py)
    python3 d1max_localizer.py --map ~/d1max_maps/lab/map --http-port 8781

Method is Monte-Carlo localisation against the 2-D occupancy grid that
d1max_slam.save_map() already produces:

  odometry delta  -> particle motion update (+ noise)
  lidar endpoints -> likelihood-field scoring against a distance transform
                     of the occupied cells
  low-variance resampling, weighted-mean pose out

numpy only -- no scipy, no nav2 install on the robot.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    HAVE_ROS = True
except ImportError:                      # allows --selftest off-robot
    HAVE_ROS = False
    Node = object

# RoboSense Airy extrinsics, from docs/source/2.10 and the URDF.
FRONT_XYZ = (0.4043, 0.0, -0.0377)
REAR_XYZ = (-0.4043, 0.0, -0.0377)

Z_MIN, Z_MAX = 0.15, 1.20        # same band d1max_slam uses to build the grid
N_BEAMS = 72                     # angular bins scored per update
MAX_RANGE = 40.0


# ----------------------------------------------------------------- map load
def load_map(stem: str):
    """Read a nav2-style PGM+YAML pair written by d1max_slam.write_pgm/yaml."""
    yaml_path = stem + ".yaml" if not stem.endswith(".yaml") else stem
    base = os.path.dirname(os.path.abspath(yaml_path))
    meta = {}
    for line in open(yaml_path):
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        meta[k.strip()] = v.strip()

    res = float(meta["resolution"])
    origin = [float(x) for x in meta["origin"].strip("[]").split(",")[:2]]
    pgm = os.path.join(base, meta["image"])

    with open(pgm, "rb") as fh:
        assert fh.readline().strip() == b"P5", "not a binary PGM"
        dims = fh.readline().split()
        while dims[0].startswith(b"#"):
            dims = fh.readline().split()
        w, h = int(dims[0]), int(dims[1])
        fh.readline()                                   # maxval
        img = np.frombuffer(fh.read(w * h), dtype=np.uint8).reshape(h, w)

    # write_pgm(): 0 = occupied, 254 = free. Row 0 is max y.
    occupied = img < 128
    return occupied, res, origin, w, h


def chamfer_dt(occupied: np.ndarray) -> np.ndarray:
    """Euclidean-ish distance (in cells) to the nearest occupied cell."""
    INF = 1e9
    d = np.where(occupied, 0.0, INF).astype(np.float64)
    h, w = d.shape
    D, S = 1.0, math.sqrt(2.0)

    for r in range(h):                                   # forward pass
        row = d[r]
        if r > 0:
            up = d[r - 1]
            np.minimum(row, up + D, out=row)
            np.minimum(row[1:], up[:-1] + S, out=row[1:])
            np.minimum(row[:-1], up[1:] + S, out=row[:-1])
        for c in range(1, w):
            if row[c] > row[c - 1] + D:
                row[c] = row[c - 1] + D

    for r in range(h - 1, -1, -1):                       # backward pass
        row = d[r]
        if r < h - 1:
            dn = d[r + 1]
            np.minimum(row, dn + D, out=row)
            np.minimum(row[1:], dn[:-1] + S, out=row[1:])
            np.minimum(row[:-1], dn[1:] + S, out=row[:-1])
        for c in range(w - 2, -1, -1):
            if row[c] > row[c + 1] + D:
                row[c] = row[c + 1] + D
    return d


# ------------------------------------------------------------------- filter
class MCL:
    def __init__(self, occupied, res, origin, w, h, n=600, sigma=0.25,
                 beta=0.22, rough=0.35):
        self.res, self.origin, self.w, self.h = res, origin, w, h
        self.dt = chamfer_dt(occupied) * res             # metres to nearest wall
        self.n = n
        self.sigma = sigma
        # Lidar beams are strongly correlated, so summing one log-likelihood
        # per beam overcounts the evidence and collapses the particle set to
        # a single point. Temper the sum, and roughen after resampling, or the
        # filter goes overconfident and can no longer track odometry drift.
        self.beta = beta
        self.rough = rough
        self.p = np.zeros((n, 3))
        self.wt = np.full(n, 1.0 / n)
        self.ready = False

    def seed(self, x, y, yaw, spread=0.5, yaw_spread=0.35):
        self.p[:, 0] = x + np.random.normal(0, spread, self.n)
        self.p[:, 1] = y + np.random.normal(0, spread, self.n)
        self.p[:, 2] = yaw + np.random.normal(0, yaw_spread, self.n)
        self.wt[:] = 1.0 / self.n
        self.ready = True

    def predict(self, dx, dy, dth):
        """Odometry delta expressed in the robot's own frame."""
        dist = math.hypot(dx, dy)
        n = self.n
        nd = dist * np.random.normal(1.0, 0.12, n) + np.random.normal(0, 0.01, n)
        nt = dth * np.random.normal(1.0, 0.12, n) + np.random.normal(0, 0.012, n)
        head = np.arctan2(dy, dx) if dist > 1e-6 else 0.0
        th = self.p[:, 2] + head
        self.p[:, 0] += nd * np.cos(th)
        self.p[:, 1] += nd * np.sin(th)
        self.p[:, 2] = np.arctan2(np.sin(self.p[:, 2] + nt),
                                  np.cos(self.p[:, 2] + nt))

    def update(self, angles, ranges):
        good = np.isfinite(ranges) & (ranges > 0.3) & (ranges < MAX_RANGE)
        if good.sum() < 8:
            return
        a, r = angles[good], ranges[good]

        c, s = np.cos(self.p[:, 2]), np.sin(self.p[:, 2])
        # endpoints for every particle x every beam
        bx = r[None, :] * np.cos(a)[None, :]
        by = r[None, :] * np.sin(a)[None, :]
        ex = self.p[:, 0:1] + c[:, None] * bx - s[:, None] * by
        ey = self.p[:, 1:2] + s[:, None] * bx + c[:, None] * by

        col = ((ex - self.origin[0]) / self.res).astype(np.int32)
        row = (self.h - 1 - ((ey - self.origin[1]) / self.res)).astype(np.int32)
        inside = (col >= 0) & (col < self.w) & (row >= 0) & (row < self.h)
        col = np.clip(col, 0, self.w - 1)
        row = np.clip(row, 0, self.h - 1)

        dist = self.dt[row, col]
        dist = np.where(inside, dist, 3.0)              # off-map = poor support
        logp = -(dist ** 2) / (2.0 * self.sigma ** 2)
        ll = logp.sum(axis=1) * self.beta

        ll -= ll.max()
        wt = np.exp(ll)
        tot = wt.sum()
        if tot <= 0 or not np.isfinite(tot):
            return
        self.wt = wt / tot
        if 1.0 / np.sum(self.wt ** 2) < self.n * 0.5:   # effective sample size
            self._resample()

    def _resample(self):
        pos = (np.arange(self.n) + np.random.uniform()) / self.n
        idx = np.searchsorted(np.cumsum(self.wt), pos)
        idx = np.clip(idx, 0, self.n - 1)
        self.p = self.p[idx]
        self.wt[:] = 1.0 / self.n
        # Roughening: reintroduce diversity scaled to the current spread,
        # with a floor so a collapsed set can still recover.
        sx = max(self.p[:, 0].std(), 0.02)
        sy = max(self.p[:, 1].std(), 0.02)
        st = max(self.p[:, 2].std(), 0.01)
        k = self.rough * self.n ** (-1.0 / 3.0)
        self.p[:, 0] += np.random.normal(0, max(k * sx, 0.015), self.n)
        self.p[:, 1] += np.random.normal(0, max(k * sy, 0.015), self.n)
        self.p[:, 2] += np.random.normal(0, max(k * st, 0.008), self.n)

    def pose(self):
        x = float(np.average(self.p[:, 0], weights=self.wt))
        y = float(np.average(self.p[:, 1], weights=self.wt))
        yaw = float(math.atan2(np.average(np.sin(self.p[:, 2]), weights=self.wt),
                               np.average(np.cos(self.p[:, 2]), weights=self.wt)))
        spread = float(np.sqrt(np.average((self.p[:, 0] - x) ** 2 +
                                          (self.p[:, 1] - y) ** 2,
                                          weights=self.wt)))
        return x, y, yaw, spread


# --------------------------------------------------------------- PointCloud2
def cloud_to_xyz(msg) -> np.ndarray:
    """PointCloud2 -> Nx3 float32, using the message's own field offsets."""
    off = {f.name: (f.offset, f.datatype) for f in msg.fields}
    for k in ("x", "y", "z"):
        if k not in off:
            return np.empty((0, 3), np.float32)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    n = len(raw) // msg.point_step
    raw = raw[: n * msg.point_step].reshape(n, msg.point_step)
    out = np.empty((n, 3), np.float32)
    for i, k in enumerate(("x", "y", "z")):
        o, _ = off[k]
        out[:, i] = raw[:, o:o + 4].copy().view(np.float32).ravel()
    return out


def to_beams(pts: np.ndarray, ex: tuple) -> tuple[np.ndarray, np.ndarray]:
    """Sensor-frame cloud -> nearest range per angular bin, in the base frame."""
    if pts.size == 0:
        return np.empty(0), np.empty(0)
    x = pts[:, 0] + ex[0]
    y = pts[:, 1] + ex[1]
    z = pts[:, 2] + ex[2]
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > Z_MIN) & (z < Z_MAX)
    x, y = x[ok], y[ok]
    if x.size == 0:
        return np.empty(0), np.empty(0)
    ang = np.arctan2(y, x)
    rng = np.hypot(x, y)
    bins = ((ang + math.pi) / (2 * math.pi) * N_BEAMS).astype(np.int32) % N_BEAMS
    best = np.full(N_BEAMS, np.inf)
    np.minimum.at(best, bins, rng)
    centres = (np.arange(N_BEAMS) + 0.5) / N_BEAMS * 2 * math.pi - math.pi
    return centres, best


# ------------------------------------------------------------------ ROS node
class LocalizerNode(Node):
    def __init__(self, mcl: MCL, use_rear: bool):
        super().__init__("d1max_localizer")
        self.mcl = mcl
        self.lock = threading.Lock()
        self.last_odom = None
        self.pending = {}
        self.updates = 0
        self.last_update = 0.0

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, "/front_lidar",
                                 lambda m: self.on_cloud(m, FRONT_XYZ), qos)
        if use_rear:
            self.create_subscription(PointCloud2, "/rear_lidar",
                                     lambda m: self.on_cloud(m, REAR_XYZ), qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.get_logger().info("localizer up; waiting for a seed pose")

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        cur = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        with self.lock:
            if self.last_odom is not None and self.mcl.ready:
                px, py, pth = self.last_odom
                dxw, dyw = cur[0] - px, cur[1] - py
                # rotate the world-frame delta into the robot frame
                c, s = math.cos(-pth), math.sin(-pth)
                dx, dy = c * dxw - s * dyw, s * dxw + c * dyw
                dth = math.atan2(math.sin(cur[2] - pth), math.cos(cur[2] - pth))
                if abs(dx) + abs(dy) + abs(dth) > 1e-4:
                    self.mcl.predict(dx, dy, dth)
            self.last_odom = cur

    def on_cloud(self, msg, extrinsic):
        ang, rng = to_beams(cloud_to_xyz(msg), extrinsic)
        if ang.size == 0:
            return
        with self.lock:
            self.pending[extrinsic] = (ang, rng)
            if not self.mcl.ready:
                return
            # fuse whichever units reported since the last update
            allang = np.concatenate([a for a, _ in self.pending.values()])
            allrng = np.concatenate([r for _, r in self.pending.values()])
            self.pending.clear()
            self.mcl.update(allang, allrng)
            self.updates += 1
            self.last_update = time.time()

    def snapshot(self) -> dict:
        with self.lock:
            if not self.mcl.ready:
                return {"ready": False, "reason": "not seeded"}
            x, y, yaw, spread = self.mcl.pose()
            return {
                "ready": True, "frame": "map",
                "x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 4),
                "spread": round(spread, 3),
                "converged": spread < 0.6,
                "updates": self.updates,
                "age": round(time.time() - self.last_update, 2)
                if self.last_update else None,
            }

    def seed(self, x, y, yaw):
        with self.lock:
            self.mcl.seed(x, y, yaw)
            self.last_odom = None
        self.get_logger().info(f"seeded at ({x:.2f}, {y:.2f}, {yaw:.2f})")


def serve(node: "LocalizerNode", port: int):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _j(self, obj, code=200):
            b = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(b)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path.startswith("/pose"):
                self._j(node.snapshot())
            else:
                self._j({"error": "not found"}, 404)

        def do_POST(self):
            if not self.path.startswith("/seed"):
                return self._j({"error": "not found"}, 404)
            n = int(self.headers.get("Content-Length") or 0)
            try:
                b = json.loads(self.rfile.read(n) or b"{}")
                node.seed(float(b.get("x", 0)), float(b.get("y", 0)),
                          float(b.get("yaw", 0)))
                self._j({"ok": True, "pose": node.snapshot()})
            except Exception as exc:
                self._j({"ok": False, "error": str(exc)}, 400)

    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


# ----------------------------------------------------------------- self-test
def selftest() -> int:
    """Synthetic room + simulated drive: does the filter actually converge?"""
    print("building a synthetic 20x14 m room ...")
    res, w, h = 0.05, 400, 280
    occ = np.zeros((h, w), bool)
    occ[0, :] = occ[-1, :] = True
    occ[:, 0] = occ[:, -1] = True
    occ[80:200, 200] = True                       # an interior wall for texture
    origin = [0.0, 0.0]

    def raycast(x, y, yaw, n=N_BEAMS):
        ang = (np.arange(n) + 0.5) / n * 2 * math.pi - math.pi
        out = np.full(n, np.inf)
        for i, a in enumerate(ang):
            ca, sa = math.cos(yaw + a), math.sin(yaw + a)
            for d in np.arange(0.2, 25.0, res):
                c = int((x + ca * d - origin[0]) / res)
                r = int(h - 1 - (y + sa * d - origin[1]) / res)
                if not (0 <= c < w and 0 <= r < h):
                    break
                if occ[r, c]:
                    out[i] = d
                    break
        return ang, out

    mcl = MCL(occ, res, origin, w, h, n=600)
    true = [5.0, 5.0, 0.0]
    mcl.seed(true[0] + 0.4, true[1] - 0.4, true[2] + 0.25)   # deliberately off

    errs = []
    for step in range(28):
        dx, dth = 0.22, (0.06 if step > 12 else 0.0)
        true[2] += dth
        true[0] += dx * math.cos(true[2])
        true[1] += dx * math.sin(true[2])
        # odometry the filter sees is biased, exactly like real drift
        mcl.predict(dx * 1.04, 0.0, dth * 1.04)
        mcl.update(*raycast(*true))
        x, y, yaw, spread = mcl.pose()
        err = math.hypot(x - true[0], y - true[1])
        errs.append(err)
        if step % 7 == 0:
            print(f"  step {step:2d}  true=({true[0]:5.2f},{true[1]:5.2f})  "
                  f"est=({x:5.2f},{y:5.2f})  err={err:.3f} m  spread={spread:.2f}")

    final = sum(errs[-5:]) / 5
    print(f"\nmean error over the last 5 steps: {final:.3f} m")
    ok = final < 0.30
    print("PASS -- filter tracks the robot" if ok else
          "FAIL -- filter did not converge")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map", help="path to the map .yaml (or its stem)")
    ap.add_argument("--http-port", type=int, default=8781)
    ap.add_argument("--particles", type=int, default=600)
    ap.add_argument("--rear", action="store_true", help="also fuse /rear_lidar")
    ap.add_argument("--seed", nargs=3, type=float, metavar=("X", "Y", "YAW"))
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.map:
        print("--map is required (or use --selftest)")
        return 2
    if not HAVE_ROS:
        print("rclpy not importable -- this must run on the Orin NX")
        return 2

    occ, res, origin, w, h = load_map(args.map)
    print(f"map {w}x{h} @ {res} m, origin {origin}, "
          f"{int(occ.sum())} occupied cells")

    mcl = MCL(occ, res, origin, w, h, n=args.particles)
    rclpy.init()
    node = LocalizerNode(mcl, use_rear=args.rear)
    if args.seed:
        node.seed(*args.seed)
    serve(node, args.http_port)
    print(f"pose on http://0.0.0.0:{args.http_port}/pose  (POST /seed to set)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

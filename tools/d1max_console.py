#!/usr/bin/env python3
"""D1 Max operator console -- local web UI over the ZSKJ protocol.

Runs a small HTTP server on your machine that owns the UDP connection to the
robot and serves a browser UI. The browser never touches the socket: it
publishes setpoints and presses buttons, and the client layer in
d1max_client.py decides what actually goes on the wire and at what rate.

    # 1. connect your laptop to the robot's Wi-Fi (XG2WIFI_xxxxxx / 12345678)
    python3 tools/d1max_console.py --host 192.168.234.1

    # wired instead
    python3 tools/d1max_console.py --host 192.168.168.168

    # no robot to hand -- runs against the built-in simulator
    python3 tools/d1max_console.py --sim

Then open http://127.0.0.1:8770 . Use --bind 0.0.0.0 to reach it from a phone
on the same network, which is also how you try the mobile layout.

Stdlib only: no pip install, no build step.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import d1max_proto as p          # noqa: E402
import d1max_mission as mission  # noqa: E402
import d1max_slam as slam      # noqa: E402
from d1max_client import RobotClient  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
UI_FILE = os.path.join(HERE, "console_ui.html")

SNAPSHOT_HZ = 10

# Known robot endpoints, used to tell "wrong network" from "robot not answering".
AP_HOST = "192.168.234.1"
WIRED_HOST = "192.168.168.168"
ORIN_HOST = "192.168.168.100"

client: RobotClient | None = None
_sim_proc: subprocess.Popen | None = None
executor: "mission.MissionExecutor | None" = None
calib: "mission.AxisCalibration | None" = None

# Recording buffer and a rolling pose trace for the map view.
recording: dict | None = None
trace: list = []
TRACE_MAX = 4000
_trace_lock = threading.Lock()


def _trace_thread() -> None:
    """Sample the odometry pose at 10 Hz so the map has something to draw."""
    last = None
    while True:
        time.sleep(0.1)
        c = client
        if c is None or c.state != "CONNECTED":
            continue
        pose = c.pose()
        if pose is None:
            continue
        # Only keep points that actually moved, so a parked robot does not
        # fill the buffer with duplicates.
        if last and abs(pose[0] - last[0]) < 0.02 and abs(pose[1] - last[1]) < 0.02:
            continue
        last = pose
        with _trace_lock:
            trace.append([round(pose[0], 3), round(pose[1], 3)])
            if len(trace) > TRACE_MAX:
                del trace[:len(trace) - TRACE_MAX]


def attach_executor(c: RobotClient) -> None:
    global executor, calib
    executor = mission.MissionExecutor(
        c, on_log=lambda label, detail: c.log("sys", label, detail))
    calib = mission.AxisCalibration(
        c, on_log=lambda label, detail: c.log("sys", label, detail))


def route_source_ip(dest: str) -> str | None:
    """Local address the OS would use to reach ``dest``, or None if no route.

    Connecting a UDP socket sends nothing -- it only does the route lookup --
    so this is a free way to ask "am I on the robot's network yet?".
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((dest, 9))
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


def netcheck(target: str) -> dict:
    """Where this machine sits relative to the robot, in plain terms."""
    src = route_source_ip(target)
    ap = route_source_ip(AP_HOST)
    wired = route_source_ip(WIRED_HOST)

    on_ap = bool(ap and ap.startswith("192.168.234."))
    on_wired = bool(wired and wired.startswith("192.168.168."))
    loopback = target.startswith("127.")

    if loopback:
        where, hint = "simulator", "Pointed at a local simulator, not a robot."
    elif on_ap and target == AP_HOST:
        where, hint = "ap", f"On the robot's Wi-Fi (local address {ap}). Ready to connect."
    elif on_wired and target in (WIRED_HOST, ORIN_HOST):
        where, hint = "wired", f"On the robot's wired subnet (local address {wired}). Ready to connect."
    elif on_ap and target == WIRED_HOST:
        where = "ap-need-route"
        hint = ("On the Wi-Fi AP but targeting the wired address. Either use "
                f"{AP_HOST}, or add the route: "
                "sudo ip route add 192.168.168.0/24 via 192.168.234.1")
    elif src is None:
        where, hint = "no-route", f"No route to {target}. Not on the robot's network yet."
    else:
        where = "other"
        hint = (f"Local address for {target} would be {src}, which is not a robot "
                "subnet. Join the robot's Wi-Fi (XG2WIFI_xxxxxx / 12345678).")

    return {
        "target": target,
        "source_ip": src,
        "on_ap": on_ap,
        "on_wired": on_wired,
        "where": where,
        "hint": hint,
        "ready": where in ("ap", "wired", "simulator"),
    }


def full_state() -> dict:
    """Client snapshot plus mission/recording/calibration state."""
    if client is None:
        return {"state": "DISCONNECTED", "connected": False}
    s = client.snapshot()
    pose = client.pose()
    s["pose"] = ([round(v, 3) for v in pose] if pose else None)
    s["mission"] = executor.status() if executor else {"state": "IDLE"}
    s["calib"] = ({"state": calib.state, "result": calib.result} if calib
                  else {"state": "IDLE", "result": {}})
    if recording is not None:
        s["recording"] = {
            "name": recording["name"],
            "count": len(recording["waypoints"]),
            "waypoints": [w.as_dict() for w in recording["waypoints"]],
        }
    else:
        s["recording"] = None
    return s


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    # keep the console readable -- one line per request is noise here
    def log_message(self, fmt, *args):
        pass

    # ------------------------------------------------------------ helpers
    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, obj, code: int = 200) -> None:
        self._send(code, json.dumps(obj).encode("utf-8"), "application/json")

    def _body(self) -> dict:
        n = int(self.headers.get("Content-Length") or 0)
        if not n:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return {}

    # ---------------------------------------------------------------- GET
    def do_GET(self):
        path = urlparse(self.path).path

        if path in ("/", "/index.html"):
            try:
                with open(UI_FILE, "rb") as fh:
                    self._send(200, fh.read(), "text/html; charset=utf-8")
            except OSError:
                self._send(500, b"console_ui.html not found next to d1max_console.py",
                           "text/plain")
            return

        if path == "/favicon.ico":
            self._send(200, b"", "image/x-icon")
            return

        if path == "/api/netcheck":
            qs = urlparse(self.path).query
            target = AP_HOST
            for part in qs.split("&"):
                if part.startswith("host="):
                    target = unquote(part[5:]) or AP_HOST
            self._json(netcheck(target))
            return

        if path == "/api/state":
            self._json(full_state())
            return

        if path == "/api/missions":
            self._json({"missions": [m.as_dict() for m in mission.load_missions()]})
            return

        if path == "/api/trace":
            with _trace_lock:
                self._json({"trace": list(trace)})
            return

        if path == "/api/slam/check":
            try:
                self._json(slam.check_env(verbose=False))
            except Exception as exc:
                self._json({"ready": False, "issues": [str(exc)], "topics": []})
            return

        if path == "/api/maps":
            try:
                self._json({"maps": slam.list_maps(), "dir": slam.MAP_DIR})
            except Exception as exc:
                self._json({"maps": [], "error": str(exc)})
            return

        if path.startswith("/api/map/preview/"):
            name = unquote(path.rsplit("/", 1)[-1])
            png = os.path.join(slam.map_path(name), "preview.png")
            try:
                with open(png, "rb") as fh:
                    self._send(200, fh.read(), "image/png")
            except OSError:
                self._send(404, b"no preview", "text/plain")
            return

        if path == "/api/events":
            self._events()
            return

        self._send(404, b"not found", "text/plain")

    def _events(self) -> None:
        """Server-sent events: a state snapshot at 10 Hz plus new log lines."""
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        last_log = time.time() - 5.0
        try:
            while True:
                if client is not None:
                    payload = {"state": full_state()}
                    lines = client.recent_log(since=last_log)
                    if lines:
                        last_log = max(l["t"] for l in lines)
                        payload["log"] = lines
                else:
                    payload = {"state": {"state": "DISCONNECTED"}}

                chunk = f"data: {json.dumps(payload)}\n\n".encode("utf-8")
                self.wfile.write(chunk)
                self.wfile.flush()
                time.sleep(1.0 / SNAPSHOT_HZ)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    # --------------------------------------------------------------- POST
    def do_POST(self):
        global client
        path = urlparse(self.path).path
        body = self._body()

        if path == "/api/slam/save":
            try:
                meta = slam.save_map(
                    str(body.get("name") or "map"), body.get("pcd") or None,
                    float(body.get("res") or slam.DEFAULT_RES),
                    float(body.get("z_min") or slam.DEFAULT_Z_MIN),
                    float(body.get("z_max") or slam.DEFAULT_Z_MAX),
                    int(body.get("min_hits") or slam.DEFAULT_HITS))
                self._json({"ok": True, "meta": meta})
            except Exception as exc:
                self._json({"ok": False, "error": str(exc)})
            return

        if path == "/api/connect":
            host = body.get("host") or (client.host if client else "192.168.234.1")
            port = int(body.get("port") or p.UDP_PORT)
            if client is not None:
                client.close()
            # Short timeout when the UI is polling in "waiting for robot" mode,
            # so each attempt fails fast instead of stalling the retry loop.
            timeout = float(body.get("timeout") or 5.0)
            client = RobotClient(host=host, port=port, platform=sys.platform)
            try:
                info = client.connect(timeout=timeout)
            except Exception as exc:
                self._json({"ok": False, "error": str(exc),
                            "net": netcheck(host)}, 200)
                return
            client.enable_default_telemetry()
            attach_executor(client)
            with _trace_lock:
                trace.clear()
            self._json({"ok": True, "handshake": info})
            return

        if client is None or not client.state == "CONNECTED":
            self._json({"ok": False, "error": "not connected"}, 200)
            return

        try:
            if path == "/api/disconnect":
                client.close()

            elif path == "/api/velocity":
                # The executor owns the setpoint while a mission runs. Two
                # writers fighting over it is how a robot ends up somewhere
                # nobody intended.
                if executor and executor.state == "RUNNING":
                    self._json({"ok": False, "error": "mission running -- pause or abort first"})
                    return
                client.set_velocity(
                    lx=float(body.get("lx", 0.0)), ly=float(body.get("ly", 0.0)),
                    rx=float(body.get("rx", 0.0)), ry=float(body.get("ry", 0.0)),
                    turn=str(body.get("turn", "none")),
                    high_low=str(body.get("high_low", "none")))

            elif path == "/api/stop":
                client.stop()

            elif path == "/api/command":
                client.command(str(body.get("cmd", "")))

            elif path == "/api/estop":
                client.emergency_stop(bool(body.get("on", True)))

            elif path == "/api/take_control":
                client.take_control()

            elif path == "/api/release_control":
                client.release_control()

            elif path == "/api/sensor":
                client.sensor_config(int(body["sensor"]), bool(body["enable"]),
                                     body.get("freq"))

            # ---------------- recording ----------------
            elif path == "/api/record/start":
                globals()["recording"] = {
                    "name": str(body.get("name") or "Recorded route"),
                    "waypoints": [],
                }
                with _trace_lock:
                    trace.clear()
                client.log("sys", "recording started", recording["name"])

            elif path == "/api/record/mark":
                if recording is None:
                    self._json({"ok": False, "error": "not recording"})
                    return
                pose = client.pose()
                if pose is None:
                    self._json({"ok": False, "error": "no pose -- is sensor 30 enabled?"})
                    return
                bs = client.body_state or {}
                light = (bs.get("fill_light") or {}).get("front")
                kind = str(body.get("kind") or "transit")
                wp = mission.Waypoint(
                    x=pose[0], y=pose[1], yaw=pose[2],
                    name=str(body.get("name") or f"WP {len(recording['waypoints']) + 1}"),
                    kind=kind,
                    tolerance=float(body.get("tolerance")
                                    or (0.15 if kind == "capture" else 0.30)),
                    hold_heading=bool(body.get("hold_heading", kind == "capture")),
                    dwell_s=float(body.get("dwell_s") or (2.0 if kind == "capture" else 0.0)),
                    lights=("on" if light else "keep"))
                recording["waypoints"].append(wp)
                client.log("sys", "waypoint marked",
                           f"{wp.name} @ {wp.x:.2f},{wp.y:.2f}")
                self._json({"ok": True, "count": len(recording["waypoints"]),
                            "waypoint": wp.as_dict()})
                return

            elif path == "/api/record/undo":
                if recording and recording["waypoints"]:
                    recording["waypoints"].pop()

            elif path == "/api/record/finish":
                if recording is None or not recording["waypoints"]:
                    globals()["recording"] = None
                    self._json({"ok": False, "error": "nothing recorded"})
                    return
                m = mission.Mission(name=recording["name"],
                                    waypoints=recording["waypoints"])
                mission.save_mission(m)
                client.log("sys", "mission saved",
                           f"{m.name} · {len(m.waypoints)} waypoints")
                globals()["recording"] = None
                self._json({"ok": True, "mission": m.as_dict()})
                return

            elif path == "/api/record/cancel":
                globals()["recording"] = None

            # ---------------- mission ----------------
            elif path == "/api/mission/run":
                target = str(body.get("id") or "")
                found = next((m for m in mission.load_missions() if m.id == target), None)
                if found is None:
                    self._json({"ok": False, "error": "mission not found"})
                    return
                ok, why = executor.start(found)
                self._json({"ok": ok, "error": why})
                return

            elif path == "/api/mission/abort":
                executor.abort(str(body.get("reason") or "operator abort"))

            elif path == "/api/mission/pause":
                executor.pause()

            elif path == "/api/mission/resume":
                executor.resume()

            elif path == "/api/mission/delete":
                mission.delete_mission(str(body.get("id") or ""))

            # ---------------- calibration ----------------
            elif path == "/api/calibrate":
                ok, why = calib.start(gain=float(body.get("gain") or 0.12),
                                      seconds=float(body.get("seconds") or 2.5))
                self._json({"ok": ok, "error": why})
                return

            else:
                self._json({"ok": False, "error": "unknown endpoint"}, 404)
                return

        except Exception as exc:
            self._json({"ok": False, "error": str(exc)}, 200)
            return

        self._json({"ok": True})


def start_sim(port: int) -> subprocess.Popen:
    proc = subprocess.Popen(
        [sys.executable, os.path.join(HERE, "d1max_sim.py"),
         "--host", "127.0.0.1", "--port", str(port), "--quiet"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    return proc


def main() -> int:
    global client, _sim_proc

    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.234.1",
                    help="robot address (default: the Wi-Fi AP address)")
    ap.add_argument("--port", type=int, default=p.UDP_PORT,
                    help=f"robot UDP port (default {p.UDP_PORT}; WebSocket 8081 is not used here)")
    ap.add_argument("--sim", action="store_true",
                    help="start the built-in simulator and point the console at it")
    ap.add_argument("--sim-port", type=int, default=8099)
    ap.add_argument("--bind", default="127.0.0.1",
                    help="console HTTP bind address; use 0.0.0.0 to reach it from a phone")
    ap.add_argument("--http-port", type=int, default=8770)
    ap.add_argument("--connect", action="store_true",
                    help="connect to the robot on startup instead of waiting for the UI")
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    host, port = args.host, args.port
    if args.sim:
        _sim_proc = start_sim(args.sim_port)
        host, port = "127.0.0.1", args.sim_port
        print(f"[console] simulator running on {host}:{port}")

    client = RobotClient(host=host, port=port, platform=sys.platform)
    attach_executor(client)
    threading.Thread(target=_trace_thread, daemon=True, name="d1max-trace").start()

    if args.connect or args.sim:
        try:
            info = client.connect()
            client.enable_default_telemetry()
            print(f"[console] connected: sn={info.get('sn')} "
                  f"device_type={info.get('device_type')}")
        except Exception as exc:
            print(f"[console] initial connect failed: {exc}")
            print("[console] the UI is still up -- press CONNECT there to retry")

    srv = ThreadingHTTPServer((args.bind, args.http_port), Handler)
    srv.daemon_threads = True

    shown = "127.0.0.1" if args.bind in ("127.0.0.1", "localhost") else args.bind
    url = f"http://{shown}:{args.http_port}"
    print(f"[console] UI at {url}")
    print(f"[console] robot target {host}:{port}")
    if args.bind == "127.0.0.1":
        print("[console] (use --bind 0.0.0.0 to open this on a phone)")
    print("[console] Ctrl-C to stop")

    if not args.no_browser and args.bind in ("127.0.0.1", "localhost"):
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()

    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[console] shutting down")
    finally:
        if executor is not None and executor.running:
            executor.abort("console shutting down")
        if client is not None:
            client.close()
        srv.server_close()
        if _sim_proc is not None:
            _sim_proc.terminate()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

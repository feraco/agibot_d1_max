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
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import d1max_proto as p          # noqa: E402
from d1max_client import RobotClient  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
UI_FILE = os.path.join(HERE, "console_ui.html")

SNAPSHOT_HZ = 10

client: RobotClient | None = None
_sim_proc: subprocess.Popen | None = None


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

        if path == "/api/state":
            self._json(client.snapshot() if client else {"state": "DISCONNECTED"})
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
                    payload = {"state": client.snapshot()}
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

        if path == "/api/connect":
            host = body.get("host") or (client.host if client else "192.168.234.1")
            port = int(body.get("port") or p.UDP_PORT)
            if client is not None:
                client.close()
            client = RobotClient(host=host, port=port, platform=sys.platform)
            try:
                info = client.connect()
            except Exception as exc:
                self._json({"ok": False, "error": str(exc)}, 200)
                return
            client.enable_default_telemetry()
            self._json({"ok": True, "handshake": info})
            return

        if client is None or not client.state == "CONNECTED":
            self._json({"ok": False, "error": "not connected"}, 200)
            return

        try:
            if path == "/api/disconnect":
                client.close()

            elif path == "/api/velocity":
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
        if client is not None:
            client.close()
        srv.server_close()
        if _sim_proc is not None:
            _sim_proc.terminate()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

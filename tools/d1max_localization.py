#!/usr/bin/env python3
"""Console-side manager for map-frame localisation running on the Orin NX.

Starts d1max_localizer.py remotely, polls its HTTP pose endpoint, and exposes
a pose_provider with the same (x, y, yaw) contract as RobotClient.pose() so the
mission executor can be switched between drifting odometry and the SLAM map
without knowing the difference.
"""

from __future__ import annotations

import json
import shlex
import threading
import time
import urllib.request

import d1max_map as M
from d1max_mapsession import MappingError, _remote, _ros

LOCALIZER_PORT = 8781
STALE_S = 1.5           # older than this and we report no pose, never a stale one


class Localization:
    def __init__(self) -> None:
        self._lk = threading.Lock()
        self.map_name: str | None = None
        self.running = False
        self.last: dict | None = None
        self.last_at = 0.0
        self.error: str | None = None
        self._poll: threading.Thread | None = None
        self._stop = threading.Event()

    # ------------------------------------------------------------- remote
    def start(self, map_name: str, seed=(0.0, 0.0, 0.0), rear: bool = True) -> dict:
        map_name = (map_name or "").strip()
        if not map_name or any(c in map_name for c in "/ \\'\"$`"):
            raise MappingError("pick a saved map by name (no slashes or spaces)")

        with self._lk:
            if self.running:
                raise MappingError(f"already localising against '{self.map_name}'")

        rc, _ = _remote("echo ok", timeout=20)
        if rc != 0:
            raise MappingError(
                f"cannot reach the Orin NX at {M.ORIN_HOST}. Add the route:\n"
                "    sudo ip route add 192.168.168.0/24 via 192.168.234.1")

        # The map must exist on the robot; fetch pulls it to the laptop, so push
        # it back if the console has it and the Orin does not.
        remote_map = f"{M.REMOTE_MAPS}/{map_name}/map.yaml"
        rc, out = _remote(f"test -f {remote_map} && echo yes || echo no", timeout=20)
        if not out.strip().endswith("yes"):
            raise MappingError(
                f"no map at {remote_map} on the Orin NX. Build one first "
                f"(record -> build), or scp the map/ directory across.")

        for f in M.PUSH_FILES:
            try:
                M.scp_to(f"{M.HERE}/{f}", f"{M.REMOTE_DIR}/")
            except Exception:
                pass

        _remote("pkill -f d1max_localizer || true", timeout=20)
        cmd = (f"cd {M.REMOTE_DIR} && nohup python3 d1max_localizer.py "
               f"--map {remote_map} --http-port {LOCALIZER_PORT} "
               f"{'--rear' if rear else ''} "
               f"--seed {seed[0]} {seed[1]} {seed[2]} "
               f"> /tmp/d1max_localizer.log 2>&1 &")
        _ros(cmd, timeout=40)
        time.sleep(5.0)

        snap = self._fetch()
        if snap is None:
            _, log = _remote("tail -25 /tmp/d1max_localizer.log 2>/dev/null || true",
                             timeout=20)
            raise MappingError(f"localizer did not come up. Remote log:\n{log}")

        with self._lk:
            self.map_name = map_name
            self.running = True
            self.error = None
        self._stop.clear()
        self._poll = threading.Thread(target=self._loop, daemon=True,
                                      name="d1max-localize")
        self._poll.start()
        return self.status()

    def stop(self) -> dict:
        self._stop.set()
        _remote("pkill -f d1max_localizer || true", timeout=20)
        with self._lk:
            self.running = False
        return self.status()

    def seed(self, x: float, y: float, yaw: float) -> dict:
        body = json.dumps({"x": x, "y": y, "yaw": yaw}).encode()
        req = urllib.request.Request(
            f"http://{M.ORIN_HOST}:{LOCALIZER_PORT}/seed", data=body,
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())

    # -------------------------------------------------------------- polling
    def _fetch(self) -> dict | None:
        try:
            with urllib.request.urlopen(
                    f"http://{M.ORIN_HOST}:{LOCALIZER_PORT}/pose", timeout=3) as r:
                return json.loads(r.read())
        except Exception:
            return None

    def _loop(self) -> None:
        while not self._stop.is_set():
            snap = self._fetch()
            with self._lk:
                if snap is not None:
                    self.last, self.last_at = snap, time.time()
                elif time.time() - self.last_at > 5.0:
                    self.error = "no response from the localizer on the Orin NX"
            self._stop.wait(0.2)

    # ------------------------------------------------------------ contract
    def pose(self) -> tuple[float, float, float] | None:
        """Same shape as RobotClient.pose(). None when not trustworthy."""
        with self._lk:
            s, at = self.last, self.last_at
        if not s or not s.get("ready") or (time.time() - at) > STALE_S:
            return None
        return (float(s["x"]), float(s["y"]), float(s["yaw"]))

    def status(self) -> dict:
        with self._lk:
            s = dict(self.last or {})
            s.update({
                "running": self.running,
                "map": self.map_name,
                "error": self.error,
                "age": round(time.time() - self.last_at, 2) if self.last_at else None,
            })
            return s

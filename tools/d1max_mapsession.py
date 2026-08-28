#!/usr/bin/env python3
"""Non-interactive mapping sessions, so the console can start and stop a run.

``d1max_map.py record`` is built for a terminal: it prompts, then streams until
Ctrl-C. The web console needs the same recording driven programmatically, so
this wraps the remote side as an object with start/status/stop.

Recording happens on the Orin NX exactly as in d1max_map -- your laptop needs no
ROS 2 and no Zenoh, which sidesteps both the conda shadowing problem and the
rmw_zenoh version split between the robot's Humble (0.1.9) and a 24.04 host's
Jazzy (0.2.9).
"""

from __future__ import annotations

import os
import shlex
import subprocess
import threading
import time

import d1max_map as M

BAG_PAT = "ros2 bag record"


class MappingError(RuntimeError):
    pass


def _remote(cmd: str, timeout: float = 45) -> tuple[int, str]:
    """run_remote, but never SystemExit -- d1max_map.die() would kill the server."""
    try:
        return M.run_remote(cmd, timeout=timeout)
    except SystemExit as exc:
        raise MappingError(str(exc) or "ssh unavailable") from exc


def _ros(cmd: str, timeout: float = 45) -> tuple[int, str]:
    try:
        return M.ros_remote(cmd, timeout=timeout)
    except SystemExit as exc:
        raise MappingError(str(exc) or "ssh unavailable") from exc


class MappingSession:
    """One recording run on the Orin NX. Safe to poll from HTTP handlers."""

    def __init__(self) -> None:
        self._lk = threading.Lock()
        self.name: str | None = None
        self.started: float = 0.0
        self.stopped: float = 0.0
        self.topics: list[str] = []
        self.odom_ok: bool | None = None
        self.last_error: str | None = None
        self._starting = False

    # ------------------------------------------------------------ helpers
    @property
    def bag_dir(self) -> str:
        return f"{M.REMOTE_MAPS}/{self.name}/bag"

    def _recorder_alive(self) -> bool:
        rc, out = _remote(f"pgrep -f {shlex.quote(BAG_PAT)} >/dev/null && echo yes || echo no",
                          timeout=20)
        return out.strip().endswith("yes")

    def _bag_bytes(self) -> int:
        rc, out = _remote(f"du -sb {self.bag_dir} 2>/dev/null | cut -f1 || echo 0", timeout=20)
        try:
            return int((out.strip().splitlines() or ["0"])[-1])
        except ValueError:
            return 0

    # -------------------------------------------------------------- start
    def start(self, name: str, rear: bool = False, with_odom: bool = True) -> dict:
        name = (name or "").strip() or "map"
        if any(c in name for c in "/ \\'\"$`"):
            raise MappingError("map name must not contain spaces, quotes or slashes")

        with self._lk:
            if self.running():
                raise MappingError(f"already recording '{self.name}'")
            self._starting = True
            self.last_error = None

        try:
            rc, _ = _remote("echo ok", timeout=20)
            if rc != 0:
                raise MappingError(
                    f"cannot reach the Orin NX at {M.ORIN_HOST}. On the hotspot you also "
                    "need the wired-subnet route:\n"
                    "    sudo ip route add 192.168.168.0/24 via 192.168.234.1")

            _remote(f"mkdir -p {M.REMOTE_DIR} {M.REMOTE_MAPS}")

            self.odom_ok = None
            if with_odom:
                self.odom_ok = self._start_odom_bridge()

            topics = list(M.TOPICS) + (["/odom"] if with_odom else [])
            if rear:
                topics.append("/rear_lidar")

            self.name = name
            self.topics = topics
            _remote(f"rm -rf {self.bag_dir}")

            inner = (f"cd {M.REMOTE_MAPS}/{name} && "
                     f"ros2 bag record -o bag {' '.join(topics)}")
            launch = (f"mkdir -p {M.REMOTE_MAPS}/{name} && "
                      f"nohup bash -lc {shlex.quote(M.ROS_ENV + ' && ' + inner)} "
                      f"> /tmp/d1max_bag.log 2>&1 & echo started")
            rc, out = _remote(launch, timeout=30)
            if rc != 0:
                raise MappingError(f"could not start the recorder: {out}")

            time.sleep(3.0)
            if not self._recorder_alive():
                _, log = _remote("tail -20 /tmp/d1max_bag.log 2>/dev/null || true", timeout=20)
                raise MappingError(f"recorder exited immediately. Remote log:\n{log}")

            self.started = time.time()
            self.stopped = 0.0
            return self.status()
        except MappingError as exc:
            self.last_error = str(exc)
            raise
        finally:
            self._starting = False

    def _start_odom_bridge(self) -> bool:
        """Push and run the odometry bridge so /odom lands in the bag."""
        try:
            for f in M.PUSH_FILES:
                M.scp_to(os.path.join(M.HERE, f), f"{M.REMOTE_DIR}/")
        except (subprocess.CalledProcessError, SystemExit):
            return False
        _remote("pkill -f d1max_odom_bridge || true", timeout=20)
        _ros(f"cd {M.REMOTE_DIR} && nohup python3 d1max_odom_bridge.py "
             f"--host {M.RK_HOST} > /tmp/d1max_bridge.log 2>&1 &", timeout=30)
        time.sleep(4.0)
        rc, out = _ros("ros2 topic list 2>/dev/null | grep -c '^/odom$' || true", timeout=40)
        return out.strip().endswith("1")

    # --------------------------------------------------------------- stop
    def stop(self) -> dict:
        if not self.name:
            raise MappingError("no recording has been started")
        # SIGINT, not SIGKILL: rosbag2 only writes metadata.yaml on a clean
        # shutdown, and without it the bag cannot be replayed.
        _remote(f"pkill -INT -f {shlex.quote(BAG_PAT)} || true", timeout=20)
        for _ in range(15):
            time.sleep(1.0)
            if not self._recorder_alive():
                break
        else:
            _remote(f"pkill -KILL -f {shlex.quote(BAG_PAT)} || true", timeout=20)
        _remote("pkill -f d1max_odom_bridge || true", timeout=20)

        self.stopped = time.time()
        rc, out = _remote(
            f"test -f {self.bag_dir}/metadata.yaml && echo yes || echo no", timeout=20)
        st = self.status()
        st["metadata_ok"] = out.strip().endswith("yes")
        if not st["metadata_ok"]:
            st["warning"] = ("bag has no metadata.yaml -- it may not replay. "
                             "Was the recorder killed rather than interrupted?")
        return st

    # ------------------------------------------------------------- status
    def running(self) -> bool:
        return bool(self.name) and self.started > 0 and self.stopped == 0

    def status(self) -> dict:
        st: dict = {
            "name": self.name,
            "running": False,
            "starting": self._starting,
            "topics": self.topics,
            "odom_ok": self.odom_ok,
            "elapsed": 0.0,
            "bytes": 0,
            "error": self.last_error,
        }
        if not self.name:
            return st
        if self.running():
            try:
                st["running"] = self._recorder_alive()
                st["bytes"] = self._bag_bytes()
            except MappingError as exc:
                st["error"] = str(exc)
            st["elapsed"] = time.time() - self.started
        else:
            st["elapsed"] = max(0.0, self.stopped - self.started)
            try:
                st["bytes"] = self._bag_bytes()
            except MappingError:
                pass
        return st

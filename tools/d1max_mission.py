#!/usr/bin/env python3
"""Mission recording and execution for the D1 Max.

Waypoints are recorded by driving: teleop the robot along the route and mark
each point at the pose the robot is actually standing at. The executor then
drives back to those poses.

Pose comes from protocol message 1102 -- the control board's own 50 Hz
leg-kinematics + IMU estimate. That is dead reckoning: it drifts, so this is
sound for short routes and replay-soon-after-recording, not for a map that
lives for weeks. The executor only ever asks for `client.pose()`, so swapping
in a SLAM pose later changes nothing here.
See docs/dev/03-slam-mapping-plan.md.

Safety model, in order of precedence:

  1. Any abort condition immediately zeroes the setpoint.
  2. Output is clamped to conservative fractions of the LOW speed level.
  3. Losing control ownership, an e-stop, a fatal fault or a stale pose all
     abort -- none of them are recoverable in-flight.
  4. If this process dies, the robot's own 1 s command expiry stops it.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from dataclasses import dataclass, field, asdict
from typing import Any, Callable

import d1max_proto as p

MISSION_DIR = os.path.expanduser("~/.d1max/missions")

# --- controller limits, in normalised teleop units ------------------------
# At speed level LOW, 1.0 maps to 1.0 m/s forward and 1.5 rad/s yaw. These
# caps therefore mean ~0.30 m/s and ~0.37 rad/s. Deliberately slow.
MAX_FWD = 0.30
MAX_YAW = 0.25
MIN_FWD = 0.06          # below this the robot barely moves; don't creep
K_FWD = 0.55            # m -> normalised
K_YAW = 0.80            # rad -> normalised

ALIGN_RAD = 0.35        # heading error above which we rotate in place first
CONTROL_HZ = 20
POSE_STALE_S = 0.5

DEFAULT_TOL_M = 0.30
DEFAULT_HEADING_TOL = 0.25
DEFAULT_STEP_TIMEOUT = 90.0


def wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float
    name: str = ""
    kind: str = "transit"            # transit | capture
    tolerance: float = DEFAULT_TOL_M
    hold_heading: bool = False       # also match yaw on arrival
    dwell_s: float = 0.0
    lights: str = "keep"             # keep | on | off
    timeout_s: float = DEFAULT_STEP_TIMEOUT

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class Mission:
    name: str
    waypoints: list[Waypoint] = field(default_factory=list)
    speed_level: str = "low"
    mode: str = "general"
    id: str = field(default_factory=lambda: uuid.uuid4().hex[:12])
    created_at: float = field(default_factory=time.time)
    frame: str = "odom"              # honest about what the poses mean
    notes: str = ""

    def as_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["waypoints"] = [w.as_dict() for w in self.waypoints]
        return d

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "Mission":
        wps = [Waypoint(**w) for w in d.get("waypoints", [])]
        return Mission(
            name=d.get("name", "untitled"), waypoints=wps,
            speed_level=d.get("speed_level", "low"), mode=d.get("mode", "general"),
            id=d.get("id", uuid.uuid4().hex[:12]),
            created_at=d.get("created_at", time.time()),
            frame=d.get("frame", "odom"), notes=d.get("notes", ""))


# --------------------------------------------------------------- storage
def ensure_dir() -> None:
    os.makedirs(MISSION_DIR, exist_ok=True)


def save_mission(m: Mission) -> str:
    ensure_dir()
    path = os.path.join(MISSION_DIR, f"{m.id}.json")
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(m.as_dict(), fh, indent=2)
    os.replace(tmp, path)            # never leave a half-written mission
    return path


def load_missions() -> list[Mission]:
    ensure_dir()
    out = []
    for fn in sorted(os.listdir(MISSION_DIR)):
        if not fn.endswith(".json"):
            continue
        try:
            with open(os.path.join(MISSION_DIR, fn)) as fh:
                out.append(Mission.from_dict(json.load(fh)))
        except (OSError, ValueError, TypeError):
            continue
    return sorted(out, key=lambda m: m.created_at, reverse=True)


def delete_mission(mission_id: str) -> bool:
    path = os.path.join(MISSION_DIR, f"{mission_id}.json")
    try:
        os.remove(path)
        return True
    except OSError:
        return False


# --------------------------------------------------------------- executor
class MissionExecutor:
    """Drives a recorded mission. One at a time, per client."""

    def __init__(self, client, on_log: Callable[[str, str], None] | None = None):
        self.client = client
        self.on_log = on_log or (lambda label, detail: None)

        self.state = "IDLE"          # IDLE READY RUNNING PAUSED COMPLETED ABORTED
        self.mission: Mission | None = None
        self.index = 0
        self.reason = ""
        self.started_at = 0.0
        self.step_started_at = 0.0
        self.distance_to_target: float | None = None
        self.last_cmd = (0.0, 0.0)

        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._pause = threading.Event()
        self._lock = threading.Lock()

    # ------------------------------------------------------------- helpers
    @property
    def running(self) -> bool:
        return self.state == "RUNNING" or self.state == "PAUSED"

    def status(self) -> dict[str, Any]:
        m = self.mission
        return {
            "state": self.state,
            "reason": self.reason,
            "mission_id": m.id if m else None,
            "mission_name": m.name if m else None,
            "index": self.index,
            "total": len(m.waypoints) if m else 0,
            "waypoint": (m.waypoints[self.index].as_dict()
                         if m and 0 <= self.index < len(m.waypoints) else None),
            "distance": (round(self.distance_to_target, 2)
                         if self.distance_to_target is not None else None),
            "elapsed": round(time.time() - self.started_at, 1) if self.started_at else 0,
            "cmd": {"fwd": round(self.last_cmd[0], 3), "yaw": round(self.last_cmd[1], 3)},
        }

    # --------------------------------------------------------------- gates
    def _abort_reason(self) -> str | None:
        """Anything that must stop the mission right now."""
        c = self.client
        if c.state != "CONNECTED":
            return "connection lost"
        if not c.control_source in (p.CTRL_SDK, p.CTRL_EXTERNAL):
            return "control ownership lost"
        bs = c.body_state or {}
        estop = bs.get("estop", {}) or {}
        if estop.get("software") or estop.get("hardware"):
            return "emergency stop asserted"
        for f in (c.faults or []):
            if (f.get("level") or 0) >= 3:
                return f"fatal fault: {f.get('fault')}"
        if c.pose() is None:
            return "pose stale (no 1102 motion data)"
        return None

    # --------------------------------------------------------------- start
    def start(self, mission: Mission) -> tuple[bool, str]:
        with self._lock:
            if self.running:
                return False, "a mission is already running"
            if not mission.waypoints:
                return False, "mission has no waypoints"
            gate = self._abort_reason()
            if gate:
                return False, gate

            self.mission = mission
            self.index = 0
            self.reason = ""
            self.state = "RUNNING"
            self.started_at = time.time()
            self.step_started_at = time.time()
            self._stop.clear()
            self._pause.clear()
            self._thread = threading.Thread(target=self._run, daemon=True,
                                            name="d1max-mission")
            self._thread.start()
            return True, ""

    def abort(self, reason: str = "operator abort") -> None:
        self._stop.set()
        self.reason = reason
        if self.state == "RUNNING" or self.state == "PAUSED":
            self.state = "ABORTED"
        self.client.stop()
        self.on_log("MISSION ABORT", reason)

    def pause(self) -> None:
        if self.state == "RUNNING":
            self._pause.set()
            self.state = "PAUSED"
            self.client.stop()
            self.on_log("mission paused", "")

    def resume(self) -> None:
        if self.state == "PAUSED":
            gate = self._abort_reason()
            if gate:
                self.abort(gate)
                return
            self._pause.clear()
            self.state = "RUNNING"
            self.step_started_at = time.time()
            self.on_log("mission resumed", "")

    # ----------------------------------------------------------- main loop
    def _run(self) -> None:
        m = self.mission
        assert m is not None
        self.on_log("MISSION START", f"{m.name} · {len(m.waypoints)} waypoints")

        # Pin the speed level so the actuator mapping stays constant for the
        # whole run -- the controller gains assume LOW.
        try:
            self.client.command(f"speed/{m.speed_level}")
            time.sleep(0.2)
            self.client.command(f"mode/{m.mode}")
            time.sleep(0.4)
        except Exception:
            pass

        period = 1.0 / CONTROL_HZ

        while not self._stop.is_set() and self.index < len(m.waypoints):
            if self._pause.is_set():
                self.client.stop()
                time.sleep(period)
                continue

            gate = self._abort_reason()
            if gate:
                self.abort(gate)
                break

            wp = m.waypoints[self.index]

            if time.time() - self.step_started_at > wp.timeout_s:
                self.abort(f"step {self.index + 1} timed out after {wp.timeout_s:.0f}s")
                break

            pose = self.client.pose()
            if pose is None:
                self.abort("pose stale (no 1102 motion data)")
                break

            arrived, fwd, yaw, dist = self._step(pose, wp)
            self.distance_to_target = dist
            self.last_cmd = (fwd, yaw)

            if arrived:
                self._on_arrival(wp)
                self.index += 1
                self.step_started_at = time.time()
                continue

            # lx = forward/back, ly = lateral, rx = yaw. Matches the teleop UI.
            self.client.set_velocity(lx=fwd, ly=0.0, rx=yaw)
            time.sleep(period)

        self.client.stop()
        self.distance_to_target = None
        self.last_cmd = (0.0, 0.0)

        if self.state == "RUNNING":
            self.state = "COMPLETED"
            self.on_log("MISSION COMPLETE",
                        f"{m.name} · {len(m.waypoints)} waypoints · "
                        f"{time.time() - self.started_at:.0f}s")

    def _step(self, pose, wp: Waypoint):
        """One control cycle. Returns (arrived, fwd_cmd, yaw_cmd, distance)."""
        x, y, yaw = pose
        dx, dy = wp.x - x, wp.y - y
        dist = math.hypot(dx, dy)

        if dist <= wp.tolerance:
            if wp.hold_heading:
                herr = wrap(wp.yaw - yaw)
                if abs(herr) > DEFAULT_HEADING_TOL:
                    return False, 0.0, clamp(K_YAW * herr, -MAX_YAW, MAX_YAW), dist
            return True, 0.0, 0.0, dist

        bearing = math.atan2(dy, dx)
        herr = wrap(bearing - yaw)
        yaw_cmd = clamp(K_YAW * herr, -MAX_YAW, MAX_YAW)

        if abs(herr) > ALIGN_RAD:
            # Rotate in place first. Driving while badly misaligned on a legged
            # robot produces wide, unpredictable arcs.
            return False, 0.0, yaw_cmd, dist

        fwd = clamp(K_FWD * dist, MIN_FWD, MAX_FWD)
        fwd *= max(0.35, 1.0 - abs(herr) / ALIGN_RAD)   # ease off while turning
        return False, fwd, yaw_cmd, dist

    def _on_arrival(self, wp: Waypoint) -> None:
        label = wp.name or f"waypoint {self.index + 1}"
        self.on_log("waypoint reached", label)
        self.client.stop()

        if wp.lights == "on":
            self.client.command("fill_light/front_light_on")
        elif wp.lights == "off":
            self.client.command("fill_light/front_light_off")

        if wp.dwell_s > 0:
            end = time.time() + wp.dwell_s
            while time.time() < end and not self._stop.is_set():
                if self._abort_reason():
                    break
                time.sleep(0.1)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ------------------------------------------------------------- calibration
class AxisCalibration:
    """Drive forward briefly and check what the odometry says happened.

    Two things this settles, both of which silently ruin autonomy if wrong:
    whether `lx` really is the forward axis (the protocol PDF contradicts
    itself -- see docs/dev/02-wire-protocol.md), and whether 1102's position
    and rpy are consistent with each other.
    """

    def __init__(self, client, on_log=None):
        self.client = client
        self.on_log = on_log or (lambda label, detail: None)
        self.state = "IDLE"
        self.result: dict[str, Any] = {}
        self._thread: threading.Thread | None = None

    def start(self, gain: float = 0.12, seconds: float = 2.5) -> tuple[bool, str]:
        if self.state == "RUNNING":
            return False, "calibration already running"
        if self.client.pose() is None:
            return False, "no pose data -- is sensor 30 enabled?"
        self.state = "RUNNING"
        self.result = {}
        self._thread = threading.Thread(
            target=self._run, args=(gain, seconds), daemon=True)
        self._thread.start()
        return True, ""

    def _run(self, gain: float, seconds: float) -> None:
        c = self.client
        self.on_log("CALIBRATION", f"driving lx={gain} for {seconds}s -- stand clear")
        start = c.pose()
        t_end = time.time() + seconds
        try:
            while time.time() < t_end:
                if c.control_source not in (p.CTRL_SDK, p.CTRL_EXTERNAL):
                    raise RuntimeError("control ownership lost")
                c.set_velocity(lx=gain)
                time.sleep(0.05)
        except Exception as exc:
            c.stop()
            self.state = "FAILED"
            self.result = {"ok": False, "error": str(exc)}
            self.on_log("CALIBRATION FAILED", str(exc))
            return
        c.stop()
        time.sleep(0.8)                      # let it settle before reading
        end = c.pose()

        if start is None or end is None:
            self.state = "FAILED"
            self.result = {"ok": False, "error": "lost pose during calibration"}
            return

        dx, dy = end[0] - start[0], end[1] - start[1]
        moved = math.hypot(dx, dy)
        heading = start[2]
        # Component of travel along vs across the body heading at start.
        along = dx * math.cos(heading) + dy * math.sin(heading)
        across = -dx * math.sin(heading) + dy * math.cos(heading)

        if moved < 0.05:
            verdict = "no-motion"
            note = ("Robot barely moved. Is it standing, in general mode, "
                    "with control held and e-stop clear?")
        elif abs(along) > abs(across) * 1.5:
            verdict = "forward" if along > 0 else "backward"
            note = ("lx drives along the body heading -- the convention this "
                    "console assumes is correct."
                    if along > 0 else
                    "lx drove BACKWARD. Invert the forward axis.")
        elif abs(across) > abs(along) * 1.5:
            verdict = "lateral"
            note = ("lx moved the robot SIDEWAYS. lx and ly are swapped "
                    "relative to what this console assumes -- fix before "
                    "running a mission.")
        else:
            verdict = "ambiguous"
            note = "Motion was diagonal. Re-run on flat ground with more space."

        self.result = {
            "ok": verdict == "forward",
            "verdict": verdict, "note": note,
            "moved_m": round(moved, 3),
            "along_m": round(along, 3), "across_m": round(across, 3),
            "start": [round(v, 3) for v in start],
            "end": [round(v, 3) for v in end],
        }
        self.state = "DONE"
        self.on_log("CALIBRATION " + verdict.upper(),
                    f"moved {moved:.2f}m (along {along:+.2f}, across {across:+.2f})")

#!/usr/bin/env python3
"""D1 Max client: connection, safety layer, and telemetry decode.

Everything that must be correct regardless of what UI sits on top:

  * handshake, then a mandatory 5 Hz heartbeat
  * a fixed-rate teleop transmit loop -- 50 Hz commanding, 5 Hz idle
  * a watchdog that zeroes velocity when setpoints go stale
  * control-ownership tracking, treating loss as a hard stop
  * decode of body state (1004), faults (1005) and motion (1102)

The UI never touches the socket. It publishes a setpoint; this layer decides
what actually goes on the wire and how often.

Stdlib only. See docs/dev/02-wire-protocol.md.
"""

from __future__ import annotations

import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable

import d1max_proto as p

# Zero the setpoint if the UI has not refreshed it within this window. The
# robot expires commands after 1000 ms on its own; we stop well before that.
WATCHDOG_MS = 350

HEARTBEAT_HZ = 5
TELEOP_ACTIVE_HZ = 50
TELEOP_IDLE_HZ = 5


def _now() -> float:
    return time.monotonic()


@dataclass
class LogLine:
    t: float
    dir: str        # "tx" | "rx" | "sys"
    label: str
    detail: str = ""

    def as_dict(self) -> dict[str, Any]:
        return {"t": self.t, "dir": self.dir, "label": self.label, "detail": self.detail}


@dataclass
class RobotClient:
    host: str
    port: int = p.UDP_PORT
    src: int = p.SRC_SDK

    app_version: str = "0.1.0"
    device: str = "d1max-console"
    platform: str = "linux"
    package_name: str = "dev.d1max.console"

    on_log: Callable[[LogLine], None] | None = None

    # --- connection ---
    state: str = field(default="DISCONNECTED", init=False)
    handshake_info: dict[str, Any] = field(default_factory=dict, init=False)
    last_error: str = field(default="", init=False)

    # --- telemetry ---
    body_state: dict[str, Any] = field(default_factory=dict, init=False)
    motion: dict[str, Any] = field(default_factory=dict, init=False)
    motion_at: float = field(default=0.0, init=False)   # monotonic, for staleness
    faults: list[dict[str, Any]] = field(default_factory=list, init=False)
    rtt_ms: float | None = field(default=None, init=False)
    last_rx_at: float = field(default=0.0, init=False)
    rx_count: int = field(default=0, init=False)
    tx_count: int = field(default=0, init=False)

    # --- control ownership ---
    control_source: int = field(default=p.CTRL_NONE, init=False)
    control_lost_at: float | None = field(default=None, init=False)

    # --- internals ---
    _sock: socket.socket | None = field(default=None, init=False)
    _msg_id: int = field(default=0, init=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    _running: threading.Event = field(default_factory=threading.Event, init=False)
    _threads: list[threading.Thread] = field(default_factory=list, init=False)
    _setpoint: dict[str, Any] = field(
        default_factory=lambda: {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0,
                                 "turn": "none", "high_low": "none"},
        init=False)
    _setpoint_at: float = field(default=0.0, init=False)
    _hb_sent_at: dict[int, float] = field(default_factory=dict, init=False)
    _log: deque = field(default_factory=lambda: deque(maxlen=400), init=False)

    # ------------------------------------------------------------------ log
    def log(self, direction: str, label: str, detail: str = "") -> None:
        line = LogLine(t=time.time(), dir=direction, label=label, detail=detail)
        with self._lock:
            self._log.append(line)
        if self.on_log:
            try:
                self.on_log(line)
            except Exception:
                pass

    def recent_log(self, since: float = 0.0) -> list[dict[str, Any]]:
        with self._lock:
            return [l.as_dict() for l in self._log if l.t > since]

    # --------------------------------------------------------------- wire io
    def _next_id(self) -> int:
        with self._lock:
            msg_id = self._msg_id
            self._msg_id = (self._msg_id + 1) & 0xFFFF
            return msg_id

    def _send(self, payload: dict[str, Any], label: str | None = None,
              detail: str = "", quiet: bool = False) -> int:
        sock = self._sock
        if sock is None:
            raise RuntimeError("not connected")
        msg_id = self._next_id()
        sock.sendto(p.encode(msg_id, payload), (self.host, self.port))
        self.tx_count += 1
        if not quiet:
            mtype = payload.get("head", {}).get("type")
            self.log("tx", label or p.TYPE_NAMES.get(mtype, str(mtype)), detail)
        return msg_id

    # -------------------------------------------------------------- connect
    def connect(self, timeout: float = 5.0) -> dict[str, Any]:
        """Open the socket and complete the mandatory handshake."""
        if self._running.is_set():
            raise RuntimeError("already connected")

        self.state = "CONNECTING"
        self.last_error = ""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.25)

        self.log("sys", "connect", f"{self.host}:{self.port} (UDP)")
        self._send(p.handshake(self.app_version, self.device, self.platform,
                               self.package_name, src=self.src),
                   detail="protocol 1.2.0")
        self.state = "HANDSHAKING"

        deadline = _now() + timeout
        while _now() < deadline:
            try:
                data, _ = self._sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError as exc:
                self._fail(f"socket error: {exc}")
                raise
            try:
                frame, _ = p.decode(data)
            except p.ProtocolError:
                continue
            if frame.type != p.TYPE_HANDSHAKE:
                continue

            status = frame.data.get("status_code")
            if status != p.HANDSHAKE_OK:
                reason = {
                    p.HANDSHAKE_PROTOCOL_MISMATCH: "protocol version mismatch",
                    p.HANDSHAKE_ALREADY_CONTROLLED: "already controlled by another terminal",
                }.get(status, f"status_code {status}")
                self._fail(f"handshake rejected: {reason}")
                raise RuntimeError(f"handshake rejected: {reason}")

            self.handshake_info = frame.data
            self.state = "CONNECTED"
            self.log("rx", "handshake ok",
                     f"sn={frame.data.get('sn', '?')} "
                     f"type={frame.data.get('device_type', '?')}")

            self._running.set()
            self._spawn(self._rx_loop, "rx")
            self._spawn(self._heartbeat_loop, "heartbeat")
            self._spawn(self._teleop_loop, "teleop")
            return frame.data

        self._fail("no handshake response (is the robot reachable? is the App connected?)")
        raise TimeoutError("no handshake response")

    def _spawn(self, target: Callable[[], None], name: str) -> None:
        t = threading.Thread(target=target, name=f"d1max-{name}", daemon=True)
        t.start()
        self._threads.append(t)

    def _fail(self, message: str) -> None:
        self.last_error = message
        self.state = "DISCONNECTED"
        self.log("sys", "error", message)
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    # ----------------------------------------------------------- tx threads
    def _heartbeat_loop(self) -> None:
        period = 1.0 / HEARTBEAT_HZ
        while self._running.is_set():
            try:
                stamp = p.now_ms()
                payload = p.heartbeat(src=self.src)
                payload["head"]["time"] = stamp
                with self._lock:
                    self._hb_sent_at[stamp] = _now()
                    if len(self._hb_sent_at) > 32:
                        for k in sorted(self._hb_sent_at)[:16]:
                            self._hb_sent_at.pop(k, None)
                self._send(payload, quiet=True)
            except (OSError, RuntimeError):
                break
            time.sleep(period)

    def _teleop_loop(self) -> None:
        """50 Hz while commanding, 5 Hz idle, zeroed when the setpoint is stale."""
        while self._running.is_set():
            with self._lock:
                sp = dict(self._setpoint)
                age_ms = (_now() - self._setpoint_at) * 1000.0

            stale = age_ms > WATCHDOG_MS
            if stale:
                sp = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0,
                      "turn": "none", "high_low": "none"}

            active = (any(abs(sp[k]) > 1e-6 for k in ("lx", "ly", "rx", "ry"))
                      or sp["turn"] != "none" or sp["high_low"] != "none")

            try:
                self._send(p.teleop(sp["lx"], sp["ly"], sp["rx"], sp["ry"],
                                    turn=sp["turn"], high_low=sp["high_low"],
                                    src=self.src), quiet=True)
            except (OSError, RuntimeError):
                break

            time.sleep(1.0 / (TELEOP_ACTIVE_HZ if active else TELEOP_IDLE_HZ))

    # ----------------------------------------------------------- rx thread
    def _rx_loop(self) -> None:
        while self._running.is_set():
            sock = self._sock
            if sock is None:
                break
            try:
                data, _ = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                frame, _ = p.decode(data)
            except p.ProtocolError:
                continue
            self.rx_count += 1
            self.last_rx_at = _now()
            self._handle(frame)

    def _handle(self, frame: p.Frame) -> None:
        t = frame.type

        if t == p.TYPE_HEARTBEAT:
            echo = frame.time_ms
            with self._lock:
                sent = self._hb_sent_at.pop(echo, None)
            if sent is not None:
                self.rtt_ms = (_now() - sent) * 1000.0
            return

        if t == p.TYPE_BODY_STATE:
            self.body_state = frame.data
            src = frame.data.get("ctrl_source", p.CTRL_NONE)
            if src != self.control_source:
                self.log("rx", "control source",
                         f"{p.CTRL_NAMES.get(self.control_source, '?')} "
                         f"-> {p.CTRL_NAMES.get(src, '?')}")
            self.control_source = src
            return

        if t == p.TYPE_MOTION:
            self.motion = frame.data
            self.motion_at = _now()
            return

        if t == p.TYPE_FAULT:
            faults = frame.data.get("faults", [])
            self.faults = faults
            for f in faults:
                self.log("rx", f"FAULT L{f.get('level')}",
                         f"{f.get('module', '?')}/{f.get('submodule', '?')}: "
                         f"{f.get('fault', '?')}")
            return

        if t == p.TYPE_CONTROL_TAKEN:
            # Hard stop. The App preempted us; never try to re-take automatically.
            self.control_lost_at = _now()
            self.stop()
            self.log("rx", "CONTROL LOST", "App preempted the SDK -- motion halted")
            return

        if t == p.TYPE_CONTROL_RELEASED:
            self.control_lost_at = None
            self.log("rx", "control available", "ownership released")
            return

        if t in (p.TYPE_COMMAND, p.TYPE_TAKE_CONTROL, p.TYPE_RELEASE_CONTROL,
                 p.TYPE_SENSOR_CONFIG, p.TYPE_CAMERA_BITRATE):
            self.log("rx", f"ack {p.TYPE_NAMES.get(t, t)}",
                     str(frame.data)[:120])
            return

        # 1100 / 1101 / 1103 / 1104 are high-rate; hold them silently.

    # -------------------------------------------------------------- control
    def set_velocity(self, lx: float = 0.0, ly: float = 0.0,
                     rx: float = 0.0, ry: float = 0.0,
                     turn: str = "none", high_low: str = "none") -> None:
        """Publish a setpoint. The transmit loop owns the actual send rate."""
        with self._lock:
            self._setpoint = {"lx": lx, "ly": ly, "rx": rx, "ry": ry,
                              "turn": turn, "high_low": high_low}
            self._setpoint_at = _now()

    def stop(self) -> None:
        self.set_velocity()

    def pose(self) -> tuple[float, float, float] | None:
        """(x, y, yaw) in the odometry frame, or None if 1102 is stale.

        This is dead reckoning from the control board's own estimator -- good
        enough to record and replay a route, but it drifts. Treat a None here
        as a hard stop condition, never as "keep going with the last value".
        """
        if not self.motion or (_now() - self.motion_at) > 0.5:
            return None
        pos = self.motion.get("position")
        rpy = self.motion.get("rpy")
        if not pos or not rpy or len(pos) < 2 or len(rpy) < 3:
            return None
        return (float(pos[0]), float(pos[1]), float(rpy[2]))

    def command(self, cmd: str) -> None:
        self._send(p.command(cmd, src=self.src), label="cmd", detail=cmd)

    def emergency_stop(self, on: bool = True) -> None:
        self.stop()
        self.command("emergency/stop" if on else "emergency/recover")

    def take_control(self) -> None:
        self._send(p.build_asdu(p.TYPE_TAKE_CONTROL, None, src=self.src),
                   label="take_control")

    def release_control(self) -> None:
        self.stop()
        self._send(p.build_asdu(p.TYPE_RELEASE_CONTROL, None, src=self.src),
                   label="release_control")

    def sensor_config(self, sensor: int, enable: bool,
                      freq: int | None = None) -> None:
        self._send(p.sensor_config(sensor, enable, freq, src=self.src),
                   label="sensor_config",
                   detail=f"sensor={sensor} enable={enable}"
                          + (f" freq={freq}" if freq is not None else ""))

    def enable_default_telemetry(self) -> None:
        """Ask for the streams a console needs. Re-send after every reconnect --
        the robot does not remember these across sessions."""
        self.sensor_config(p.SENSOR_MOTION, True)     # 1102 @ 50 Hz
        self.sensor_config(p.SENSOR_LUX, True)        # 1101 @ 1 Hz

    # ----------------------------------------------------------- shutdown
    def close(self) -> None:
        if not self._running.is_set() and self._sock is None:
            return
        try:
            if self._sock is not None:
                self.stop()
                self._send(p.goodbye(src=self.src), label="goodbye")
        except (OSError, RuntimeError):
            pass
        self._running.clear()
        for t in self._threads:
            if t.is_alive() and t is not threading.current_thread():
                t.join(timeout=1.0)
        self._threads.clear()
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        self.state = "DISCONNECTED"
        self.log("sys", "disconnected", "")

    # ------------------------------------------------------------ snapshot
    def snapshot(self) -> dict[str, Any]:
        """Everything the UI needs, in one JSON-safe dict."""
        bs = self.body_state
        speed = bs.get("speed", {}) or {}
        battery = bs.get("battery", {}) or {}
        estop = bs.get("estop", {}) or {}
        light = bs.get("fill_light", {}) or {}

        link_age = (_now() - self.last_rx_at) if self.last_rx_at else None
        held = self.control_source in (p.CTRL_SDK, p.CTRL_EXTERNAL)

        return {
            "state": self.state,
            "connected": self.state == "CONNECTED",
            "error": self.last_error,
            "host": self.host,
            "port": self.port,
            "handshake": self.handshake_info,

            "control": {
                "source": self.control_source,
                "source_name": p.CTRL_NAMES.get(self.control_source, "?"),
                "held": held,
                "lost": self.control_lost_at is not None,
            },

            "link": {
                "rtt_ms": round(self.rtt_ms, 1) if self.rtt_ms is not None else None,
                "silent_s": round(link_age, 2) if link_age is not None else None,
                "tx": self.tx_count,
                "rx": self.rx_count,
            },

            "robot": {
                "mode": bs.get("mode"),
                "motion_status": bs.get("motion_status"),
                "speed_level": speed.get("level"),
                "head_angle": bs.get("head_angle"),
                "head_direction": bs.get("head_direction"),
                "knee": bs.get("knee"),
                "mileage": bs.get("mile_data"),
                "obstacle_avoidance": bs.get("obstacle_avoidance"),
                "vx": speed.get("x"),
                "vy": speed.get("y"),
                "vyaw": speed.get("yaw"),
                "estop_software": estop.get("software"),
                "estop_hardware": estop.get("hardware"),
                "light_front": light.get("front"),
                "light_back": light.get("back"),
                "light_auto": light.get("auto_work"),
                "motor_temp": bs.get("motor_temp", {}),
            },

            "battery": {
                "power1": battery.get("power1"), "power2": battery.get("power2"),
                "present1": battery.get("present1"), "present2": battery.get("present2"),
                "voltage1": battery.get("voltage1"), "voltage2": battery.get("voltage2"),
                "current1": battery.get("current1"), "current2": battery.get("current2"),
                "temp1": battery.get("temperature1"), "temp2": battery.get("temperature2"),
                "status1": battery.get("power_supply_status1"),
                "status2": battery.get("power_supply_status2"),
            },

            "motion": {
                "position": self.motion.get("position"),
                "rpy": self.motion.get("rpy"),
                "v_body": self.motion.get("v_body"),
                "time_stamp": self.motion.get("time_stamp"),
            },

            "faults": self.faults,
        }

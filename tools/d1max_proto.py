#!/usr/bin/env python3
"""Reference implementation of the D1 Max ZSKJ wire protocol.

Dependency-free encoder/decoder for the framing described in
``docs/protocol/Protocol-1.2.0.pdf`` and ``docs/dev/02-wire-protocol.md``,
plus a minimal UDP client showing the mandatory handshake, heartbeat and
teleop cadences.

The codec is covered by the self-test::

    python3 tools/d1max_proto.py --self-test

The ``UdpClient`` below has *not* been exercised against hardware; it is a
skeleton to port from, not a finished client. In particular the lx/ly axis
mapping is unverified -- see the note in docs/dev/02-wire-protocol.md.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable

SYNC = b"\x5aSKJ"  # 0x5A 'S' 'K' 'J'
HEADER_LEN = 16
MAX_ASDU = 65535
PROTOCOL_VERSION = "1.2.0"

UDP_PORT = 8082
WEBSOCKET_PORT = 8081

# --- src identities -------------------------------------------------------
SRC_BODY = 1
SRC_APP = 2
SRC_SDK = 3
SRC_EXTERNAL = 4

# --- message types --------------------------------------------------------
TYPE_HANDSHAKE = 1000
TYPE_HEARTBEAT = 1001
TYPE_COMMAND = 1002
TYPE_TELEOP = 1003
TYPE_BODY_STATE = 1004
TYPE_FAULT = 1005
TYPE_SENSOR_CONFIG = 1008
TYPE_TAKE_CONTROL = 1013
TYPE_RELEASE_CONTROL = 1015
TYPE_CONTROL_TAKEN = 1016
TYPE_CONTROL_RELEASED = 1017
TYPE_CAMERA_BITRATE = 1018
TYPE_GOODBYE = 1050
TYPE_IMU = 1100
TYPE_LUX = 1101
TYPE_MOTION = 1102

# --- sensor ids for TYPE_SENSOR_CONFIG ------------------------------------
SENSOR_IMU = 10
SENSOR_LUX = 20
SENSOR_MOTION = 30  # the 50 Hz odometry source
SENSOR_BODY_SPEED = 40  # the only sensor whose freq is honoured
SENSOR_JOINT_STATE = 50

# --- handshake status codes -----------------------------------------------
HANDSHAKE_OK = 0
HANDSHAKE_PROTOCOL_MISMATCH = 10
HANDSHAKE_ALREADY_CONTROLLED = 20


class ProtocolError(ValueError):
    """Raised when a buffer cannot be decoded as a valid APDU."""


@dataclass(frozen=True)
class Frame:
    """A decoded APDU: the message id plus its parsed JSON body."""

    msg_id: int
    payload: dict[str, Any]

    @property
    def type(self) -> int | None:
        return self.payload.get("head", {}).get("type")

    @property
    def src(self) -> int | None:
        return self.payload.get("head", {}).get("src")

    @property
    def time_ms(self) -> int | None:
        return self.payload.get("head", {}).get("time")

    @property
    def data(self) -> dict[str, Any]:
        return self.payload.get("data", {})


def now_ms() -> int:
    return int(time.time() * 1000)


def build_asdu(msg_type: int, data: dict[str, Any] | None = None,
               src: int = SRC_SDK, time_ms: int | None = None) -> dict[str, Any]:
    """Assemble the ``{"head": ..., "data": ...}`` envelope."""
    payload: dict[str, Any] = {
        "head": {
            "type": msg_type,
            "time": now_ms() if time_ms is None else time_ms,
            "src": src,
        }
    }
    if data is not None:
        payload["data"] = data
    return payload


def encode(msg_id: int, payload: dict[str, Any]) -> bytes:
    """Encode one APDU: 16-byte header + UTF-8 JSON ASDU."""
    if not 0 <= msg_id <= 0xFFFF:
        raise ProtocolError(f"msg_id out of range: {msg_id}")
    # separators drop whitespace; the ASDU length cap is tight at 64 KiB.
    body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    if len(body) > MAX_ASDU:
        raise ProtocolError(f"ASDU too large: {len(body)} > {MAX_ASDU}")
    return SYNC + struct.pack("<HH8x", len(body), msg_id) + body


def decode(buf: bytes) -> tuple[Frame, bytes]:
    """Decode one APDU from ``buf``.

    Returns the frame and whatever bytes follow it. Raises ProtocolError if
    the buffer does not begin with a complete, valid APDU -- callers on a
    stream transport should treat that as "need more data" only after
    checking ``len(buf) < HEADER_LEN``.
    """
    if len(buf) < HEADER_LEN:
        raise ProtocolError(f"short header: {len(buf)} < {HEADER_LEN}")
    if buf[:4] != SYNC:
        raise ProtocolError(f"bad sync word: {buf[:4]!r}")
    length, msg_id = struct.unpack_from("<HH", buf, 4)
    end = HEADER_LEN + length
    if len(buf) < end:
        raise ProtocolError(f"short body: have {len(buf) - HEADER_LEN}, need {length}")
    try:
        payload = json.loads(buf[HEADER_LEN:end].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"malformed ASDU: {exc}") from exc
    if not isinstance(payload, dict):
        raise ProtocolError("ASDU is not a JSON object")
    return Frame(msg_id=msg_id, payload=payload), buf[end:]


def decode_stream(buf: bytes) -> tuple[list[Frame], bytes]:
    """Drain as many whole APDUs as ``buf`` holds; return them and the remainder.

    For WebSocket/TCP, where frames may be split or coalesced. On UDP each
    datagram is one APDU, so use ``decode`` directly.
    """
    frames: list[Frame] = []
    while len(buf) >= HEADER_LEN:
        try:
            frame, buf = decode(buf)
        except ProtocolError:
            break  # incomplete tail, or a desync the caller must handle
        frames.append(frame)
    return frames, buf


# --- message constructors -------------------------------------------------

def handshake(version: str, device: str, platform: str, package_name: str,
              src: int = SRC_SDK) -> dict[str, Any]:
    return build_asdu(TYPE_HANDSHAKE, {
        "version": version,
        "protocol_version": PROTOCOL_VERSION,
        "device": device,
        "platform": platform,
        "package_name": package_name,
    }, src=src)


def heartbeat(src: int = SRC_SDK) -> dict[str, Any]:
    return build_asdu(TYPE_HEARTBEAT, None, src=src)


def command(cmd: str, src: int = SRC_SDK) -> dict[str, Any]:
    return build_asdu(TYPE_COMMAND, {"cmd": cmd}, src=src)


def teleop(lx: float = 0.0, ly: float = 0.0, rx: float = 0.0, ry: float = 0.0,
           turn: str = "none", high_low: str = "none",
           src: int = SRC_SDK) -> dict[str, Any]:
    """Continuous velocity command.

    Stick values are clamped here rather than trusted -- a caller bug should
    not become an uncommanded acceleration.
    """
    def clamp(v: float) -> float:
        return max(-1.0, min(1.0, float(v)))

    # Spec: when a body action is active, stick values are sent as zero.
    if turn != "none" or high_low != "none":
        lx = ly = rx = ry = 0.0
    return build_asdu(TYPE_TELEOP, {
        "lx": clamp(lx), "ly": clamp(ly), "rx": clamp(rx), "ry": clamp(ry),
        "body": {"turn": turn, "high_low": high_low},
    }, src=src)


def sensor_config(sensor: int, enable: bool, freq: int | None = None,
                  src: int = SRC_SDK) -> dict[str, Any]:
    data: dict[str, Any] = {"sensor": sensor, "enable": enable}
    if freq is not None:
        data["freq"] = freq
    return build_asdu(TYPE_SENSOR_CONFIG, data, src=src)


def goodbye(src: int = SRC_SDK) -> dict[str, Any]:
    return build_asdu(TYPE_GOODBYE, None, src=src)


# --- minimal UDP client ---------------------------------------------------

@dataclass
class UdpClient:
    """Skeleton client: handshake, 5 Hz heartbeat, watchdogged teleop loop.

    Not hardware-tested. Port the structure, not the details.
    """

    host: str
    port: int = UDP_PORT
    src: int = SRC_SDK
    watchdog_ms: int = 300

    on_frame: Callable[[Frame], None] | None = None

    _sock: socket.socket | None = field(default=None, init=False)
    _msg_id: int = field(default=0, init=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    _running: threading.Event = field(default_factory=threading.Event, init=False)
    _setpoint: tuple[float, float, float, float] = field(default=(0.0, 0.0, 0.0, 0.0), init=False)
    _setpoint_at: float = field(default=0.0, init=False)

    def _next_id(self) -> int:
        with self._lock:
            msg_id = self._msg_id
            self._msg_id = (self._msg_id + 1) & 0xFFFF
            return msg_id

    def send(self, payload: dict[str, Any]) -> int:
        if self._sock is None:
            raise RuntimeError("not connected")
        msg_id = self._next_id()
        self._sock.sendto(encode(msg_id, payload), (self.host, self.port))
        return msg_id

    def connect(self, timeout: float = 5.0, *, version: str = "0.1.0",
                device: str = "d1max-tools", platform: str = sys.platform,
                package_name: str = "dev.d1max.tools") -> dict[str, Any]:
        """Open the socket and complete the mandatory handshake."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(timeout)
        self.send(handshake(version, device, platform, package_name, src=self.src))

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                data, _ = self._sock.recvfrom(65536)
            except socket.timeout:
                break
            try:
                frame, _ = decode(data)
            except ProtocolError:
                continue
            if frame.type == TYPE_HANDSHAKE:
                status = frame.data.get("status_code")
                if status != HANDSHAKE_OK:
                    raise RuntimeError(f"handshake rejected, status_code={status}")
                self._running.set()
                threading.Thread(target=self._heartbeat_loop, daemon=True).start()
                threading.Thread(target=self._teleop_loop, daemon=True).start()
                threading.Thread(target=self._rx_loop, daemon=True).start()
                return frame.data
        raise TimeoutError("no handshake response")

    def set_velocity(self, lx: float = 0.0, ly: float = 0.0,
                     rx: float = 0.0, ry: float = 0.0) -> None:
        """Publish a setpoint. The transmit loop owns the actual send rate."""
        with self._lock:
            self._setpoint = (lx, ly, rx, ry)
            self._setpoint_at = time.monotonic()

    def _heartbeat_loop(self) -> None:
        while self._running.is_set():  # 5 Hz, mandatory
            try:
                self.send(heartbeat(src=self.src))
            except OSError:
                break
            time.sleep(0.2)

    def _teleop_loop(self) -> None:
        """50 Hz while commanding, 5 Hz when idle, zeroed if the setpoint goes stale."""
        while self._running.is_set():
            with self._lock:
                lx, ly, rx, ry = self._setpoint
                age_ms = (time.monotonic() - self._setpoint_at) * 1000.0
            if age_ms > self.watchdog_ms:
                lx = ly = rx = ry = 0.0  # watchdog: stale setpoint means stop
            active = any(abs(v) > 1e-6 for v in (lx, ly, rx, ry))
            try:
                self.send(teleop(lx, ly, rx, ry, src=self.src))
            except OSError:
                break
            time.sleep(0.02 if active else 0.2)

    def _rx_loop(self) -> None:
        assert self._sock is not None
        while self._running.is_set():
            try:
                data, _ = self._sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                frame, _ = decode(data)
            except ProtocolError:
                continue
            if frame.type == TYPE_CONTROL_TAKEN:
                # The App preempted us. Hard abort -- never try to re-take.
                self.set_velocity(0, 0, 0, 0)
            if self.on_frame is not None:
                self.on_frame(frame)

    def close(self) -> None:
        if self._sock is not None and self._running.is_set():
            try:
                self.send(goodbye(src=self.src))  # let the server drop us promptly
            except OSError:
                pass
        self._running.clear()
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def __enter__(self) -> "UdpClient":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


# --- self-test ------------------------------------------------------------

def _self_test() -> int:
    failures: list[str] = []

    def check(name: str, cond: bool) -> None:
        if cond:
            print(f"  ok    {name}")
        else:
            print(f"  FAIL  {name}")
            failures.append(name)

    print("framing")
    payload = build_asdu(TYPE_COMMAND, {"cmd": "emergency/stop"},
                         src=SRC_SDK, time_ms=1757310525622)
    wire = encode(7, payload)
    check("sync word is ZSKJ", wire[:4] == b"ZSKJ")
    check("sync word bytes are 5a 53 4b 4a", wire[:4] == bytes([0x5A, 0x53, 0x4B, 0x4A]))
    check("header is 16 bytes", len(wire) - len(json.dumps(payload, separators=(",", ":"))) == 16)
    length, msg_id = struct.unpack_from("<HH", wire, 4)
    check("length is little-endian ASDU size", length == len(wire) - HEADER_LEN)
    check("msg_id round-trips", msg_id == 7)
    check("8 reserved bytes are zero", wire[8:16] == b"\x00" * 8)

    print("decode")
    frame, rest = decode(wire)
    check("no trailing bytes", rest == b"")
    check("msg_id preserved", frame.msg_id == 7)
    check("type preserved", frame.type == TYPE_COMMAND)
    check("src preserved", frame.src == SRC_SDK)
    check("time preserved", frame.time_ms == 1757310525622)
    check("data preserved", frame.data == {"cmd": "emergency/stop"})

    print("stream reassembly")
    a = encode(1, heartbeat())
    b = encode(2, command("action/stand_up"))
    frames, tail = decode_stream(a + b)
    check("two coalesced frames decode", len(frames) == 2)
    check("ids in order", [f.msg_id for f in frames] == [1, 2])
    check("no tail", tail == b"")
    frames, tail = decode_stream(a + b[:10])
    check("partial frame is held back", len(frames) == 1)
    check("partial bytes returned as tail", tail == b[:10])

    print("msg_id wraparound")
    client = UdpClient(host="203.0.113.1")
    client._msg_id = 0xFFFF
    check("0xFFFF allocates", client._next_id() == 0xFFFF)
    check("wraps to 0", client._next_id() == 0)

    print("teleop clamping")
    t = teleop(lx=5.0, ly=-9.0, rx=0.5, ry=-0.25)["data"]
    check("lx clamps to +1", t["lx"] == 1.0)
    check("ly clamps to -1", t["ly"] == -1.0)
    check("in-range values pass through", (t["rx"], t["ry"]) == (0.5, -0.25))
    t = teleop(lx=1.0, ly=1.0, turn="left")["data"]
    check("body action zeroes sticks", (t["lx"], t["ly"]) == (0.0, 0.0))
    check("body action preserved", t["body"] == {"turn": "left", "high_low": "none"})

    print("sensor config")
    s = sensor_config(SENSOR_MOTION, True)["data"]
    check("freq omitted when unset", "freq" not in s)
    check("motion sensor id is 30", s["sensor"] == 30)
    s = sensor_config(SENSOR_BODY_SPEED, True, freq=20)["data"]
    check("freq included when set", s["freq"] == 20)

    print("rejections")
    for name, buf in [
        ("bad sync word", b"XXXX" + struct.pack("<HH8x", 2, 0) + b"{}"),
        ("short header", b"ZSKJ"),
        ("truncated body", b"ZSKJ" + struct.pack("<HH8x", 99, 0) + b"{}"),
        ("malformed JSON", b"ZSKJ" + struct.pack("<HH8x", 3, 0) + b"{ ["),
        ("non-object ASDU", b"ZSKJ" + struct.pack("<HH8x", 2, 0) + b"[]"),
    ]:
        try:
            decode(buf)
            check(f"rejects {name}", False)
        except ProtocolError:
            check(f"rejects {name}", True)

    try:
        encode(0x10000, heartbeat())
        check("rejects out-of-range msg_id", False)
    except ProtocolError:
        check("rejects out-of-range msg_id", True)

    print()
    if failures:
        print(f"{len(failures)} check(s) failed")
        return 1
    print("all checks passed")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--self-test", action="store_true",
                    help="run offline codec checks and exit")
    ap.add_argument("--host", help="robot address, e.g. 192.168.234.1")
    ap.add_argument("--port", type=int, default=UDP_PORT)
    ap.add_argument("--seconds", type=float, default=10.0,
                    help="how long to stream telemetry for")
    args = ap.parse_args()

    if args.self_test:
        return _self_test()
    if not args.host:
        ap.error("--host is required unless --self-test is given")

    def show(frame: Frame) -> None:
        if frame.type in (TYPE_BODY_STATE, TYPE_FAULT, TYPE_CONTROL_TAKEN,
                          TYPE_CONTROL_RELEASED):
            print(f"[{frame.type}] {json.dumps(frame.data)[:200]}")

    with UdpClient(host=args.host, port=args.port, on_frame=show) as client:
        info = client.connect()
        print(f"connected: {json.dumps(info)}")
        client.send(sensor_config(SENSOR_MOTION, True))
        time.sleep(args.seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

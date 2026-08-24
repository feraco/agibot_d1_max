#!/usr/bin/env python3
"""A fake D1 Max body, speaking the ZSKJ protocol over UDP.

Enough of the robot to exercise a client end to end with no hardware:
handshake, heartbeat echo, command acks, sensor-report config, control
ownership, and the 1004 / 1102 telemetry streams. It integrates teleop
setpoints into a pose, so driving in the UI moves the simulated robot.

    python3 tools/d1max_sim.py --port 8099

It is a test fixture, not a model of the robot's dynamics. It will not tell
you whether your motion commands are safe -- only whether your client speaks
the protocol correctly.
"""

from __future__ import annotations

import argparse
import math
import random
import socket
import threading
import time

import d1max_proto as p

# Speed-level scaling, from docs/dev/01-platform-architecture.md.
SPEED_MAX = {
    "slow":   {"fwd": 1.0, "lat": 0.5, "yaw": 1.5},
    "medium": {"fwd": 2.0, "lat": 0.5, "yaw": 1.5},
    "high":   {"fwd": 3.0, "lat": 0.5, "yaw": 1.5},
}

MODE_FOR_CMD = {
    "mode/general": "general", "mode/in_place": "in_place",
    "mode/navigation": "nav", "mode/stair": "stair",
    "mode/follow": "follow", "mode/track": "track",
}

MOTION_FOR_CMD = {
    "action/stand_up": "stand_up", "action/crawl": "crawl",
    "action/lie_down": "lie_down", "action/locked": "locked",
    "action/climb": "climb", "action/dsb": "dsb", "action/slim": "slim",
    "action/gait_walk": "gait_walk", "action/new1new": "new1new",
}

JOINTS = [f"{s}{n}" for s in ("fl", "fr", "bl", "br") for n in range(1, 5)]


class Sim:
    def __init__(self, host: str, port: int, *, device_type: str = "ZSM-1-Pro",
                 app_holds_control: bool = False, verbose: bool = True):
        self.addr = (host, port)
        self.device_type = device_type
        self.verbose = verbose

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(self.addr)
        self.sock.settimeout(0.2)

        self.client: tuple[str, int] | None = None
        self.running = threading.Event()
        self.lock = threading.Lock()
        self.msg_id = 0

        # robot state
        self.mode = "general"
        self.motion_status = "lie_down"
        self.speed_level = "slow"
        self.head_angle = 0.0
        self.head_direction = "head"
        self.knee = "medial_facing"
        self.mileage = 15.8
        self.estop_sw = False
        self.light_front = False
        self.light_back = False
        self.light_auto = False
        self.ctrl_source = p.CTRL_APP if app_holds_control else p.CTRL_NONE

        # teleop integration
        self.sp = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}
        self.sp_at = 0.0
        self.pos = [0.0, 0.0, 0.42]
        self.yaw = 0.0
        self.v_body = [0.0, 0.0, 0.0]

        # sensor enables -- nothing streams until asked
        self.enabled = {p.SENSOR_IMU: False, p.SENSOR_LUX: False,
                        p.SENSOR_MOTION: False, p.SENSOR_BODY_SPEED: False,
                        p.SENSOR_JOINT_STATE: False}

        self.battery = 91.0

    # ------------------------------------------------------------------ io
    def say(self, *a) -> None:
        if self.verbose:
            print("[sim]", *a, flush=True)

    def send(self, payload: dict) -> None:
        if self.client is None:
            return
        with self.lock:
            mid = self.msg_id
            self.msg_id = (self.msg_id + 1) & 0xFFFF
        try:
            self.sock.sendto(p.encode(mid, payload), self.client)
        except OSError:
            pass

    def reply(self, msg_type: int, data: dict | None = None,
              time_ms: int | None = None) -> None:
        self.send(p.build_asdu(msg_type, data, src=p.SRC_BODY, time_ms=time_ms))

    # ------------------------------------------------------------- handlers
    def handle(self, frame: p.Frame, addr) -> None:
        t = frame.type

        if t == p.TYPE_HANDSHAKE:
            self.client = addr
            ver = frame.data.get("protocol_version")
            if ver != p.PROTOCOL_VERSION:
                self.say(f"handshake from {addr} REJECTED (protocol {ver})")
                self.reply(p.TYPE_HANDSHAKE, {"status_code": p.HANDSHAKE_PROTOCOL_MISMATCH})
                return
            if self.ctrl_source == p.CTRL_APP:
                self.say(f"handshake from {addr} -- App holds control, SDK may observe only")
            else:
                self.say(f"handshake from {addr} OK ({frame.data.get('package_name')})")
            self.reply(p.TYPE_HANDSHAKE, {
                "status_code": p.HANDSHAKE_OK,
                "sn": "SIM0000000001",
                "ssid": "XG2WIFI_SIM001",
                "model": "ZS1-W",
                "version": {"system": "0.2.1", "charging_pile": "0.0.0"},
                "device_type": self.device_type,
            })
            return

        if t == p.TYPE_HEARTBEAT:
            self.reply(p.TYPE_HEARTBEAT, None, time_ms=frame.time_ms)
            return

        if t == p.TYPE_TELEOP:
            d = frame.data
            with self.lock:
                self.sp = {k: float(d.get(k, 0.0)) for k in ("lx", "ly", "rx", "ry")}
                self.sp_at = time.monotonic()
            return

        if t == p.TYPE_COMMAND:
            self.do_command(frame.data.get("cmd", ""))
            self.reply(p.TYPE_COMMAND, {"cmd": frame.data.get("cmd")})
            return

        if t == p.TYPE_SENSOR_CONFIG:
            s = frame.data.get("sensor")
            en = bool(frame.data.get("enable"))
            if s in self.enabled:
                self.enabled[s] = en
                self.say(f"sensor {s} -> {'on' if en else 'off'}")
            self.reply(p.TYPE_SENSOR_CONFIG, {"sensor": s, "enable": en})
            return

        if t == p.TYPE_TAKE_CONTROL:
            # Mirrors the real body: only the App may preempt. An SDK client
            # asking while the App holds control is refused.
            if self.ctrl_source == p.CTRL_APP:
                self.reply(p.TYPE_TAKE_CONTROL,
                           {"error_code": 1, "reason": "App holds control"})
                self.say("take_control REFUSED -- App holds control")
            else:
                self.ctrl_source = p.CTRL_SDK
                self.reply(p.TYPE_TAKE_CONTROL, {"error_code": 0, "reason": ""})
                self.say("take_control granted -> SDK")
            return

        if t == p.TYPE_RELEASE_CONTROL:
            self.ctrl_source = p.CTRL_NONE
            self.reply(p.TYPE_RELEASE_CONTROL, {"error_code": 0, "reason": ""})
            self.say("control released")
            return

        if t == p.TYPE_GOODBYE:
            self.say("client said goodbye")
            self.reply(p.TYPE_GOODBYE, None)
            self.client = None
            return

    def do_command(self, cmd: str) -> None:
        self.say(f"cmd {cmd}")
        if cmd == "emergency/stop":
            self.estop_sw = True
            self.motion_status = "lie_down"
        elif cmd == "emergency/recover":
            self.estop_sw = False
        elif cmd in MODE_FOR_CMD:
            self.mode = MODE_FOR_CMD[cmd]
            # Real behaviour: a mode switch stands the robot up first.
            if self.motion_status in ("lie_down", "crawl", "locked"):
                self.motion_status = "stand_up"
        elif cmd in MOTION_FOR_CMD:
            self.motion_status = MOTION_FOR_CMD[cmd]
        elif cmd.startswith("speed/"):
            self.speed_level = {"low": "slow"}.get(cmd.split("/")[1], cmd.split("/")[1])
        elif cmd == "reverse_head_tail":
            self.head_direction = "tail" if self.head_direction == "head" else "head"
        elif cmd == "fill_light/front_light_on":
            self.light_front = True
        elif cmd == "fill_light/front_light_off":
            self.light_front = False
        elif cmd == "fill_light/back_light_on":
            self.light_back = True
        elif cmd == "fill_light/back_light_off":
            self.light_back = False
        elif cmd == "fill_light/light_auto_work_on":
            self.light_auto = True
        elif cmd == "fill_light/light_auto_work_off":
            self.light_auto = False
        elif cmd.startswith("knee_mode/"):
            self.knee = {"same_direction": "same",
                         "medial_facing": "medial_facing"}[cmd.split("/")[1]]

    # -------------------------------------------------------------- physics
    def integrate(self, dt: float) -> None:
        with self.lock:
            sp = dict(self.sp)
            age = time.monotonic() - self.sp_at

        # Mirror the robot's own 1 s command expiry.
        if age > 1.0 or self.estop_sw or self.motion_status in ("lie_down", "locked"):
            sp = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}

        lim = SPEED_MAX.get(self.speed_level, SPEED_MAX["slow"])
        vf = sp["lx"] * lim["fwd"]
        vl = sp["ly"] * lim["lat"]
        w = sp["rx"] * lim["yaw"]

        if self.mode == "in_place":
            vf = vl = 0.0
            self.head_angle = max(-45.0, min(45.0, self.head_angle + sp["rx"] * 40.0 * dt))
            w = 0.0

        self.yaw += w * dt
        self.pos[0] += (vf * math.cos(self.yaw) - vl * math.sin(self.yaw)) * dt
        self.pos[1] += (vf * math.sin(self.yaw) + vl * math.cos(self.yaw)) * dt
        self.v_body = [vf, vl, 0.0]
        self.mileage += abs(vf) * dt
        self.battery = max(0.0, self.battery - 0.0004 * dt * (1 + abs(vf)))

    # -------------------------------------------------------------- streams
    def body_state(self) -> dict:
        return {
            "temp": 0,
            "head_angle": round(self.head_angle, 3),
            "head_direction": self.head_direction,
            "knee": self.knee,
            "mode": self.mode,
            "motion_status": self.motion_status,
            "obstacle_avoidance": False,
            "mile_data": round(self.mileage, 3),
            "fill_light": {"front": self.light_front, "back": self.light_back,
                           "auto_work": self.light_auto, "display_status": False},
            "estop": {"software": self.estop_sw, "hardware": False},
            "speed": {"level": self.speed_level,
                      "x": round(self.v_body[0], 3),
                      "y": round(self.v_body[1], 3),
                      "yaw": round(self.sp["rx"] * 1.5, 3)},
            "battery": {
                "power1": round(self.battery, 1), "power2": 0,
                "present1": True, "present2": False,
                "current1": -1.77, "current2": 0,
                "voltage1": 60.9, "voltage2": 0,
                "temperature1": 26, "temperature2": 0,
                "power_supply_status1": 2, "power_supply_status2": 0,
            },
            "motor_temp": {j: 28 + random.randint(0, 4) for j in JOINTS},
            "ctrl_source": self.ctrl_source,
            "charging_pile": {"connect": False},
            "uwb": {"connect": False},
            "silence": {"enable": False},
        }

    def motion_data(self) -> dict:
        cy, sy = math.cos(self.yaw / 2), math.sin(self.yaw / 2)
        return {
            "quat": [round(cy, 6), 0.0, 0.0, round(sy, 6)],
            "rpy": [0.0, 0.0, round(self.yaw, 6)],
            "position": [round(v, 6) for v in self.pos],
            "v_world": [round(self.v_body[0] * math.cos(self.yaw), 6),
                        round(self.v_body[0] * math.sin(self.yaw), 6), 0.0],
            "v_body": [round(v, 6) for v in self.v_body],
            "omega_world": [0.0, 0.0, round(self.sp["rx"] * 1.5, 6)],
            "omega_body": [0.0, 0.0, round(self.sp["rx"] * 1.5, 6)],
            "acc": [0.0, 0.0, 9.81],
            "gyro": [0.0, 0.0, round(self.sp["rx"] * 1.5, 6)],
            "time_stamp": time.time_ns(),
        }

    # ------------------------------------------------------------- mainloop
    def _stream_loop(self) -> None:
        last_1004 = last_1102 = last_1101 = 0.0
        last_tick = time.monotonic()
        while self.running.is_set():
            now = time.monotonic()
            self.integrate(now - last_tick)
            last_tick = now

            if self.client is not None:
                if now - last_1004 >= 1.0:
                    last_1004 = now
                    self.reply(p.TYPE_BODY_STATE, self.body_state())
                if self.enabled[p.SENSOR_MOTION] and now - last_1102 >= 0.02:
                    last_1102 = now
                    self.reply(p.TYPE_MOTION, self.motion_data())
                if self.enabled[p.SENSOR_LUX] and now - last_1101 >= 1.0:
                    last_1101 = now
                    self.reply(p.TYPE_LUX, {"lux": round(180 + random.random() * 90, 1)})

            time.sleep(0.005)

    def serve(self) -> None:
        self.running.set()
        threading.Thread(target=self._stream_loop, daemon=True).start()
        self.say(f"listening on {self.addr[0]}:{self.addr[1]} "
                 f"(device_type={self.device_type})")
        try:
            while self.running.is_set():
                try:
                    data, addr = self.sock.recvfrom(65536)
                except socket.timeout:
                    continue
                except OSError:
                    break
                try:
                    frame, _ = p.decode(data)
                except p.ProtocolError:
                    continue
                self.handle(frame, addr)
        except KeyboardInterrupt:
            pass
        finally:
            self.running.clear()
            self.sock.close()
            self.say("stopped")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8099)
    ap.add_argument("--device-type", default="ZSM-1-Pro")
    ap.add_argument("--app-holds-control", action="store_true",
                    help="simulate the App already holding control, so the SDK is refused")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    Sim(args.host, args.port, device_type=args.device_type,
        app_holds_control=args.app_holds_control, verbose=not args.quiet).serve()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

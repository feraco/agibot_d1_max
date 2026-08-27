#!/usr/bin/env python3
"""ROS 2 bridge: /cmd_vel -> protocol teleop (message 1003).

The counterpart to d1max_odom_bridge.py. That one gives a SLAM/nav stack the
pose it needs; this one lets the stack actually drive. Together they are the
whole gap between "the robot streams sensors" and "nav2 can run on it" --
the D1 Max SDK provides neither (see docs/dev/08, "Obstacle avoidance and
SLAM").

Run it on the Orin NX, next to the planner:

    python3 d1max_cmdvel_bridge.py --host 192.168.168.168

Two conversions matter and both are easy to get wrong.

1. UNITS. nav2 emits Twist in m/s and rad/s. The robot wants normalised
   +/-1.0, rescaled by the *speed level* -- so 0.5 means 0.5 m/s at LOW and
   1.5 m/s at HIGH. We convert using the documented table (docs/source/3.3)
   and publish the resulting real limits so the planner can be configured to
   match rather than guess.

2. AXES. The wire fields are lx/ly/rx. The C++ SDK's Move() takes
   (left_right, forward_back, yaw) -- lateral FIRST -- while this repo's
   tooling has always assumed lx=forward. One of those is wrong and only
   hardware settles it. Run:

       python3 -c "from d1max_mission import AxisCalibration; ..."
       # or the CALIBRATE button in the console

   and pass --swap-xy if it reports the axes are transposed. Until you have
   done that on the actual robot, do not run this bridge near anything you
   would mind hitting.

Safety, in order of precedence -- any one of these zeroes the setpoint:
  * ownership lost (message 1016) -- the App preempted us
  * emergency stop active, or a FatalError fault
  * the robot is not in General mode (Move does nothing in In-Place/Stair)
  * no /cmd_vel for --cmd-timeout seconds (the planner died or stalled)

Note we deliberately do NOT defeat the robot's own 1-second Move expiry, and
d1max_client's 350 ms teleop watchdog stays in force underneath this. If this
process is killed mid-traverse the robot coasts to a stop on its own. That is
the single most valuable safety property in the stack; keep it.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import d1max_proto as p              # noqa: E402
from d1max_client import RobotClient   # noqa: E402

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    HAVE_ROS = True
except ImportError as exc:
    HAVE_ROS = False
    _ROS_ERR = str(exc)
    Node = object                     # see d1max_odom_bridge.py for why


# Speed-level scaling, from docs/source/3.3. The lateral and yaw ceilings are
# *conditional on forward speed* at MEDIUM and HIGH -- above 1 m/s forward the
# robot silently refuses to strafe at all, and yaw authority drops in steps.
# A planner that does not know this will keep issuing commands that quietly do
# nothing, which reads as the robot ignoring obstacles.
# Fault level at or above which we refuse to drive. Matches the threshold
# d1max_mission.py already uses. NOTE: the wire encoding of this field is not
# documented anywhere in the SDK docs, and the C++ FaultLevel enum runs the
# other way (FatalError=1, Error=2, Warn=3). If the wire field uses the C++
# encoding then this threshold catches warnings and misses fatal errors.
# Resolve it by provoking a real fault on hardware -- see docs/dev/09.
FATAL_FAULT_LEVEL = 3

SPEED_NAMES = {1: "LOW", 2: "MEDIUM", 3: "HIGH"}
FWD_MAX = {1: 1.0, 2: 2.0, 3: 3.0}


def lateral_max(level: int, fwd_mps: float) -> float:
    """Lateral ceiling in m/s given the commanded forward speed."""
    if level == 1:
        return 0.5
    return 0.5 if abs(fwd_mps) < 1.0 else 0.0


def yaw_max(level: int, fwd_mps: float) -> float:
    """Yaw ceiling in rad/s given the commanded forward speed."""
    a = abs(fwd_mps)
    if level == 1:
        return 1.5
    if level == 2:
        return 1.5 if a < 1.0 else 1.0
    if a < 1.0:
        return 1.5
    return 1.0 if a < 2.0 else 0.5


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class CmdVelBridge(Node):
    def __init__(self, client: RobotClient, args):
        super().__init__("d1max_cmdvel_bridge")
        self.client = client
        self.level = args.speed_level
        self.swap_xy = args.swap_xy
        self.invert_fwd = args.invert_forward
        self.invert_yaw = args.invert_yaw
        self.cmd_timeout = args.cmd_timeout
        self.allow_any_mode = args.allow_any_mode

        # Hard ceilings on top of the robot's own. Autonomy should be slower
        # than a human driver, not faster -- the planner cannot see as well.
        self.cap_fwd = args.max_fwd
        self.cap_lat = args.max_lateral
        self.cap_yaw = args.max_yaw

        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, args.topic, self.on_cmd, qos)
        self.status_pub = self.create_publisher(String, "~/status", 10)

        self.last_cmd = None
        self.last_cmd_at = 0.0
        self.blocked_reason = None
        self.sent = 0
        self.blocked = 0

        self.create_timer(1.0 / args.rate, self.tick)
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f"subscribed {args.topic}; speed level {self.level} "
            f"({SPEED_NAMES.get(self.level, '?')})")
        self.announce_limits()

    # ------------------------------------------------------------ limits
    def announce_limits(self):
        """Tell the operator what to put in the planner's config.

        Guessing these is the most common way to get a nav stack that
        oscillates: it plans for a velocity the robot will never produce.
        """
        f = min(self.cap_fwd, FWD_MAX[self.level])
        lat = min(self.cap_lat, lateral_max(self.level, 0.0))
        yaw = min(self.cap_yaw, yaw_max(self.level, 0.0))
        self.get_logger().info(
            "configure the planner with: "
            f"max_vel_x={f:.2f} max_vel_y={lat:.2f} max_vel_theta={yaw:.2f}")
        if self.level != 1:
            self.get_logger().warn(
                f"speed level {self.level}: lateral is CUT TO ZERO above "
                "1.0 m/s forward and yaw authority drops. Level 1 is "
                "strongly recommended for autonomy.")

    # -------------------------------------------------------------- input
    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_at = time.monotonic()

    # ------------------------------------------------------------- safety
    def gate(self) -> str | None:
        """Return a reason to refuse, or None to allow driving.

        Deliberately mirrors MissionExecutor._abort_reason() -- one definition
        of "unsafe to drive", used by both the waypoint executor and the nav
        stack, so they cannot disagree about it.
        """
        c = self.client
        if c.state != "CONNECTED":
            return f"link {c.state}"
        if c.control_source not in (p.CTRL_SDK, p.CTRL_EXTERNAL):
            return "control ownership lost"
        bs = c.body_state or {}
        estop = bs.get("estop", {}) or {}
        if estop.get("software") or estop.get("hardware"):
            return "emergency stop asserted"
        for f in (c.faults or []):
            if (f.get("level") or 0) >= FATAL_FAULT_LEVEL:
                return f"fatal fault: {f.get('fault')}"
        if c.pose() is None:
            return "pose stale (no 1102 motion data)"
        if not self.allow_any_mode:
            mode = str(bs.get("mode") or "")
            # Move() is General-mode only. An absent mode field is not proof
            # of General, but refusing on it would make the bridge unusable
            # against firmware that omits it -- so only refuse on a positive
            # mismatch.
            if mode and mode != "general":
                return f"mode is {mode!r}, Move() only works in general"
        if self.last_cmd is None:
            return "no /cmd_vel received yet"
        if (time.monotonic() - self.last_cmd_at) > self.cmd_timeout:
            return f"/cmd_vel stale (>{self.cmd_timeout:.1f}s)"
        return None

    # ---------------------------------------------------------------- tick
    def tick(self):
        reason = self.gate()
        if reason:
            if reason != self.blocked_reason:
                self.get_logger().warn(f"holding: {reason}")
                self.blocked_reason = reason
            self.client.stop()
            self.blocked += 1
            self.publish_status(reason, 0.0, 0.0, 0.0)
            return

        if self.blocked_reason:
            self.get_logger().info("clear, driving")
            self.blocked_reason = None

        t = self.last_cmd
        fwd = clamp(float(t.linear.x), -self.cap_fwd, self.cap_fwd)
        lat = float(t.linear.y)
        yaw = float(t.angular.z)

        # Ceilings depend on the forward speed we are actually about to ask
        # for, so clamp forward first and derive the rest from it.
        fwd = clamp(fwd, -FWD_MAX[self.level], FWD_MAX[self.level])
        lat_ceil = min(self.cap_lat, lateral_max(self.level, fwd))
        yaw_ceil = min(self.cap_yaw, yaw_max(self.level, fwd))
        lat = clamp(lat, -lat_ceil, lat_ceil) if lat_ceil > 0 else 0.0
        yaw = clamp(yaw, -yaw_ceil, yaw_ceil)

        # m/s -> normalised +/-1.0
        n_fwd = fwd / FWD_MAX[self.level]
        n_lat = lat / lateral_max(self.level, fwd) if lateral_max(self.level, fwd) else 0.0
        n_yaw = yaw / yaw_max(self.level, fwd)

        if self.invert_fwd:
            n_fwd = -n_fwd
        if self.invert_yaw:
            n_yaw = -n_yaw

        # Axis assignment. Default matches the rest of this repo (lx=forward);
        # --swap-xy matches the C++ SDK's Move(left_right, forward_back, yaw).
        if self.swap_xy:
            lx, ly = n_lat, n_fwd
        else:
            lx, ly = n_fwd, n_lat

        self.client.set_velocity(lx=lx, ly=ly, rx=n_yaw)
        self.sent += 1
        self.publish_status("driving", fwd, lat, yaw)

    def publish_status(self, state: str, fwd: float, lat: float, yaw: float):
        m = String()
        m.data = (f'{{"state":"{state}","fwd_mps":{fwd:.3f},'
                  f'"lat_mps":{lat:.3f},"yaw_rps":{yaw:.3f},'
                  f'"speed_level":{self.level}}}')
        self.status_pub.publish(m)

    def report(self):
        if self.blocked and not self.sent:
            self.get_logger().warn(f"held for 5 s: {self.blocked_reason}")
        elif self.sent:
            self.get_logger().info(
                f"{self.sent} setpoints sent, {self.blocked} held")
        self.sent = self.blocked = 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.168.168",
                    help="RK3588 control board (192.168.168.168 wired / "
                         "192.168.234.1 over the robot's Wi-Fi)")
    ap.add_argument("--port", type=int, default=p.UDP_PORT)
    ap.add_argument("--topic", default="/cmd_vel")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--speed-level", type=int, default=1, choices=(1, 2, 3),
                    help="1 LOW (recommended for autonomy), 2 MEDIUM, 3 HIGH")
    ap.add_argument("--max-fwd", type=float, default=0.40,
                    help="hard forward cap in m/s, on top of the speed level")
    ap.add_argument("--max-lateral", type=float, default=0.20)
    ap.add_argument("--max-yaw", type=float, default=0.50)
    ap.add_argument("--cmd-timeout", type=float, default=0.5,
                    help="stop if /cmd_vel goes quiet for this long")
    ap.add_argument("--swap-xy", action="store_true",
                    help="lx is lateral and ly is forward (the C++ SDK's "
                         "Move() argument order). Set this if axis "
                         "calibration reports the axes are transposed.")
    ap.add_argument("--invert-forward", action="store_true")
    ap.add_argument("--invert-yaw", action="store_true")
    ap.add_argument("--allow-any-mode", action="store_true",
                    help="do not refuse when the robot is not in General mode")
    ap.add_argument("--external", action="store_true",
                    help="identify as EXTERNAL (src=4) instead of SDK (src=3)")
    ap.add_argument("--take-control", action="store_true",
                    help="request control ownership after connecting")
    args = ap.parse_args()

    if not HAVE_ROS:
        print(f"error: ROS 2 python packages not importable ({_ROS_ERR})\n\n"
              "This node needs rclpy, so it must run where ROS 2 exists --\n"
              "the Orin NX has Humble; your laptop probably does not.\n\n"
              "    ssh robot@192.168.168.100            # password: 1\n"
              "    source /opt/ros/humble/setup.bash\n"
              "    export ROS_DOMAIN_ID=24\n"
              "    export RMW_IMPLEMENTATION=rmw_zenoh_cpp\n"
              "    python3 d1max_cmdvel_bridge.py --host 192.168.168.168\n\n"
              "If you ARE on a machine with ROS 2 and still see this, a conda\n"
              "env is probably shadowing the system Python:  conda deactivate\n",
              file=sys.stderr)
        return 1

    client = RobotClient(host=args.host, port=args.port,
                         device="d1max-cmdvel-bridge",
                         src=(p.SRC_EXTERNAL if args.external else p.SRC_SDK))
    print(f"[cmdvel] connecting to {args.host}:{args.port} …")
    try:
        info = client.connect()
    except Exception as exc:
        print(f"[cmdvel] connect failed: {exc}", file=sys.stderr)
        if "already controlled" in str(exc):
            print("\nAnother client holds the session -- usually the RC handset\n"
                  "app. Close it fully, or try --external.\n", file=sys.stderr)
        return 1
    print(f"[cmdvel] connected: sn={info.get('sn')}")

    client.sensor_config(p.SENSOR_MOTION, True)   # need pose for the gate
    if args.take_control:
        client.take_control()
    time.sleep(0.3)
    client.command(f"speed/{SPEED_NAMES[args.speed_level].lower()}")

    rclpy.init()
    node = CmdVelBridge(client, args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()
        node.destroy_node()
        rclpy.shutdown()
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

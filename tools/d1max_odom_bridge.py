#!/usr/bin/env python3
"""ROS 2 bridge: protocol message 1102 -> /odom + TF odom->base_link.

This is the piece that makes off-the-shelf SLAM work on the D1 Max. The robot
publishes LiDAR and IMU on ROS 2 from the Orin NX, but no odometry -- and a
LiDAR-inertial SLAM front-end wants a motion prior. The control board already
computes one at 50 Hz (leg kinematics fused with the IMU) and ships it as
message 1102 over UDP. This node republishes that as nav_msgs/Odometry.

Run it on the Orin NX (so it does not depend on Wi-Fi), or on a laptop for
bench work:

    # on the Orin NX, talking to the RK3588 over the wired LAN
    python3 d1max_odom_bridge.py --host 192.168.168.168

    # from a laptop on the robot's Wi-Fi
    python3 d1max_odom_bridge.py --host 192.168.234.1

Needs rclpy (ROS 2 Humble). Everything else is stdlib.

Caveats you must design around -- see docs/dev/03-slam-mapping-plan.md:
  * This is dead reckoning. It drifts, especially in yaw and on slopes.
    The covariance published here is deliberately loose so a SLAM back-end
    treats it as a prior, not as truth.
  * The RK3588 and Orin NX clocks are not necessarily disciplined together.
    By default we stamp with ROS time and report the observed offset so you
    can see the drift; --use-robot-clock stamps with the robot's own ns
    timestamp instead.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import d1max_proto as p            # noqa: E402
from d1max_client import RobotClient  # noqa: E402

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped, Quaternion
    from tf2_ros import TransformBroadcaster
    HAVE_ROS = True
except ImportError as exc:      # keep the import error useful
    HAVE_ROS = False
    _ROS_ERR = str(exc)


# Loose but honest. Legged dead reckoning is decent over metres, poor over
# tens of metres, and worst in yaw.
POSE_COV_XY = 0.05
POSE_COV_Z = 0.20
POSE_COV_RP = 0.05
POSE_COV_YAW = 0.30
TWIST_COV_LIN = 0.02
TWIST_COV_ANG = 0.10


def cov6(vx, vy, vz, vr, vp, vyaw):
    m = [0.0] * 36
    for i, v in enumerate((vx, vy, vz, vr, vp, vyaw)):
        m[i * 6 + i] = v
    return m


class OdomBridge(Node):
    def __init__(self, client: RobotClient, args):
        super().__init__("d1max_odom_bridge")
        self.client = client
        self.odom_frame = args.odom_frame
        self.base_frame = args.base_frame
        self.use_robot_clock = args.use_robot_clock

        qos = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Odometry, args.topic, qos)
        self.tf = None if args.no_tf else TransformBroadcaster(self)

        self.last_stamp_ns = 0
        self.count = 0
        self.skipped = 0
        self.offset_ms = None

        self.create_timer(1.0 / args.rate, self.tick)
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f"publishing {args.topic} at {args.rate} Hz, "
            f"{self.odom_frame} -> {self.base_frame}"
            + ("" if self.tf else " (TF disabled)"))

    def tick(self):
        m = self.client.motion
        if not m:
            return
        ns = int(m.get("time_stamp") or 0)
        if ns and ns == self.last_stamp_ns:
            return                                  # nothing new since last tick
        self.last_stamp_ns = ns

        pos = m.get("position")
        quat = m.get("quat")            # documented [w, x, y, z]
        vb = m.get("v_body") or [0, 0, 0]
        wb = m.get("omega_body") or [0, 0, 0]
        if not pos or not quat or len(pos) < 3 or len(quat) < 4:
            self.skipped += 1
            return

        now = self.get_clock().now()
        if ns:
            # Track how far the robot's clock sits from this machine's.
            self.offset_ms = (now.nanoseconds - ns) / 1e6
        stamp = (rclpy.time.Time(nanoseconds=ns).to_msg()
                 if (self.use_robot_clock and ns) else now.to_msg())

        # ROS uses (x, y, z, w); the protocol documents quat as [w, x, y, z].
        q = Quaternion(x=float(quat[1]), y=float(quat[2]),
                       z=float(quat[3]), w=float(quat[0]))

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = float(pos[0])
        od.pose.pose.position.y = float(pos[1])
        od.pose.pose.position.z = float(pos[2])
        od.pose.pose.orientation = q
        od.pose.covariance = cov6(POSE_COV_XY, POSE_COV_XY, POSE_COV_Z,
                                  POSE_COV_RP, POSE_COV_RP, POSE_COV_YAW)
        od.twist.twist.linear.x = float(vb[0])
        od.twist.twist.linear.y = float(vb[1])
        od.twist.twist.linear.z = float(vb[2])
        od.twist.twist.angular.x = float(wb[0])
        od.twist.twist.angular.y = float(wb[1])
        od.twist.twist.angular.z = float(wb[2])
        od.twist.covariance = cov6(TWIST_COV_LIN, TWIST_COV_LIN, TWIST_COV_LIN,
                                   TWIST_COV_ANG, TWIST_COV_ANG, TWIST_COV_ANG)
        self.pub.publish(od)

        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])
            t.transform.rotation = q
            self.tf.sendTransform(t)

        self.count += 1

    def report(self):
        if self.count == 0:
            self.get_logger().warn(
                "no motion data yet -- is sensor 30 enabled and the robot connected?")
            return
        msg = f"published {self.count} odom msgs"
        if self.skipped:
            msg += f", {self.skipped} malformed"
        if self.offset_ms is not None:
            msg += f", robot clock offset {self.offset_ms:+.0f} ms"
        self.get_logger().info(msg)
        self.count = self.skipped = 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.168.168",
                    help="RK3588 control board address "
                         "(192.168.168.168 wired / on the Orin; "
                         "192.168.234.1 over the robot's Wi-Fi)")
    ap.add_argument("--port", type=int, default=p.UDP_PORT)
    ap.add_argument("--topic", default="/odom")
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--no-tf", action="store_true",
                    help="publish only the topic (use if another node owns odom->base_link)")
    ap.add_argument("--external", action="store_true",
                    help="identify as EXTERNAL (src=4) instead of SDK (src=3)")
    ap.add_argument("--use-robot-clock", action="store_true",
                    help="stamp with the robot's own ns timestamp instead of ROS time")
    args = ap.parse_args()

    if not HAVE_ROS:
        print(f"error: ROS 2 python packages not importable ({_ROS_ERR})\n\n"
              "Source your ROS 2 environment first:\n"
              "    source /opt/ros/humble/setup.bash\n"
              "    export ROS_DOMAIN_ID=24\n"
              "    export RMW_IMPLEMENTATION=rmw_zenoh_cpp\n", file=sys.stderr)
        return 1

    client = RobotClient(host=args.host, port=args.port, device="d1max-odom-bridge",
                         src=(p.SRC_EXTERNAL if args.external else p.SRC_SDK))
    print(f"[bridge] connecting to {args.host}:{args.port} …")
    try:
        info = client.connect()
    except Exception as exc:
        msg = str(exc)
        print(f"[bridge] connect failed: {msg}", file=sys.stderr)
        if "already controlled" in msg:
            print(
                "\nAnother terminal holds the session. This bridge only reads\n"
                "telemetry -- it never commands the robot -- but the handshake is\n"
                "refused all the same. Fix by doing one of:\n\n"
                "  1. Close the RC handset app (background it fully), then retry.\n"
                "  2. Close the operator console if it is connected; one client at\n"
                "     a time is the supported arrangement.\n"
                "  3. Try identifying as EXTERNAL rather than SDK:\n"
                f"       python3 {os.path.basename(__file__)} --host {args.host} --external\n"
                "\nIf you are running this ON the Orin, note the control board is at\n"
                f"{args.host} — over the wired LAN that is 192.168.168.168.\n",
                file=sys.stderr)
        return 1
    print(f"[bridge] connected: sn={info.get('sn')}")
    client.sensor_config(p.SENSOR_MOTION, True)     # 1102 @ 50 Hz
    time.sleep(0.3)

    rclpy.init()
    node = OdomBridge(client, args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Record picosat motion from picosat_deploy.sdf to a CSV file.

Start this before (or while) the simulation runs. One row is written per
odometry update (50 Hz sim time) once both picosats have reported for that
time stamp.

    python3 log_picosats.py                     # until Ctrl-C -> picosats.csv
    python3 log_picosats.py -o run1.csv -t 30   # stop after 30 s of sim time

Columns (SI units, world frame unless noted):
    t                       sim time [s]
    {a,b}_x, _y, _z         position [m]
    {a,b}_vx, _vy, _vz      linear velocity [m/s]
    {a,b}_speed_in_s        speed [in/s]
    {a,b}_roll,_pitch,_yaw  orientation [deg]
    {a,b}_wx, _wy, _wz      angular velocity, body frame [deg/s]
    separation              distance between picosat centers [m]
"""

import argparse
import csv
import math
import signal
import threading

from gz.msgs.odometry_pb2 import Odometry
from gz.transport import Node

SATS = ("a", "b")
FIELDS = ["t"]
for s in SATS:
    FIELDS += [f"{s}_{k}" for k in (
        "x", "y", "z", "vx", "vy", "vz", "speed_in_s",
        "roll", "pitch", "yaw", "wx", "wy", "wz")]
FIELDS.append("separation")


def rotate(q, v):
    """Rotate vector v by unit quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v); v' = v + w*t + cross(q.xyz, t)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def euler_deg(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def unpack(msg):
    p = msg.pose.position
    o = msg.pose.orientation
    q = (o.w, o.x, o.y, o.z)
    # OdometryPublisher reports twist in the body frame; convert to world.
    lin = rotate(q, (msg.twist.linear.x, msg.twist.linear.y,
                     msg.twist.linear.z))
    ang = msg.twist.angular
    return {
        "pos": (p.x, p.y, p.z),
        "row": [p.x, p.y, p.z, *lin, math.hypot(*lin) / 0.0254,
                *euler_deg(q),
                *(math.degrees(a) for a in (ang.x, ang.y, ang.z))],
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("-o", "--out", default="picosats.csv")
    ap.add_argument("-t", "--duration", type=float, default=None,
                    help="stop after this many seconds of sim time")
    args = ap.parse_args()

    lock = threading.Lock()
    done = threading.Event()
    pending = {}  # stamp -> {sat: unpacked}
    count = [0]

    f = open(args.out, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(FIELDS)

    def make_cb(sat):
        def cb(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            with lock:
                if done.is_set():
                    return
                entry = pending.setdefault(round(stamp, 6), {})
                entry[sat] = unpack(msg)
                if len(entry) < len(SATS):
                    return
                del pending[round(stamp, 6)]
                pa, pb = entry["a"]["pos"], entry["b"]["pos"]
                writer.writerow([f"{stamp:.3f}",
                                 *(f"{v:.6g}" for v in entry["a"]["row"]),
                                 *(f"{v:.6g}" for v in entry["b"]["row"]),
                                 f"{math.dist(pa, pb):.6g}"])
                count[0] += 1
                if count[0] % 250 == 0:
                    f.flush()
                    print(f"  t = {stamp:7.2f} s   rows = {count[0]}")
                if args.duration is not None and stamp >= args.duration:
                    done.set()
        return cb

    node = Node()
    for sat in SATS:
        topic = f"/model/picosat_{sat}/odometry"
        if not node.subscribe(Odometry, topic, make_cb(sat)):
            raise SystemExit(f"could not subscribe to {topic}")

    signal.signal(signal.SIGINT, lambda *_: done.set())
    print(f"Logging to {args.out} (Ctrl-C to stop). "
          "Press play in Gazebo if it is paused.")
    while not done.wait(0.2):
        pass

    with lock:
        f.close()
    print(f"Wrote {count[0]} rows to {args.out}")


if __name__ == "__main__":
    main()

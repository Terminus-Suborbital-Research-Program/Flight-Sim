#!/usr/bin/env python3
"""Run one complete picosat deployment and record it.

Starts Gazebo, starts the motion logger and camera recorder, presses play,
ejects at the requested sim time, and shuts everything down at the end.

    python3 run_deployment.py                      # 30 s run, eject at t = 2 s
    python3 run_deployment.py -t 60 -e 5 -n test1  # 60 s, eject at 5 s
    python3 run_deployment.py --headless           # no Gazebo window
    python3 run_deployment.py --keep-open          # leave Gazebo open at the end

Output goes to runs/<name>/ (default name: date and time):
    motion.csv    picosat motion, see log_picosats.py
    images/       camera frames, see record_cameras.py
    run.json      run settings
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROS_SETUP = "/opt/ros/lyrical/setup.bash"
WORLD = "microgravity_world"
START_TIMEOUT = 60  # wall seconds to wait for Gazebo to load

# Gazebo's Python bindings need the ROS environment; load it if it isn't set.
try:
    from gz.msgs.empty_pb2 import Empty
    from gz.msgs.world_stats_pb2 import WorldStatistics
    from gz.transport import Node
except ImportError:
    if os.environ.get("RUN_DEPLOYMENT_SOURCED"):
        raise
    os.environ["RUN_DEPLOYMENT_SOURCED"] = "1"
    os.execvp("bash", ["bash", "-c",
                       f'source {ROS_SETUP} && exec python3 "$0" "$@"',
                       str(Path(__file__).resolve()), *sys.argv[1:]])


def topics():
    return subprocess.run(["gz", "topic", "-l"], capture_output=True,
                          text=True, check=False).stdout.split()


def wait_for_topic(topic, sim):
    deadline = time.monotonic() + START_TIMEOUT
    while topic not in topics():
        if sim.poll() is not None:
            raise SystemExit("Gazebo exited during startup")
        if time.monotonic() > deadline:
            raise SystemExit(f"timed out waiting for {topic}")
        time.sleep(0.5)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("-t", "--duration", type=float, default=30.0,
                    help="sim seconds to record (default 30)")
    ap.add_argument("-e", "--eject", type=float, default=2.0,
                    help="sim time to eject at (default 2)")
    ap.add_argument("-n", "--name", default=None, help="run folder name")
    ap.add_argument("--headless", action="store_true",
                    help="run without the Gazebo window")
    ap.add_argument("--keep-open", action="store_true",
                    help="leave Gazebo running after recording ends")
    args = ap.parse_args()
    if not 0 <= args.eject < args.duration:
        ap.error("--eject must be at least 0 and less than --duration")
    if "/stats" in topics():
        raise SystemExit("A Gazebo sim is already running. Close it first.")

    name = args.name or datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out = HERE / "runs" / name
    if out.exists():
        raise SystemExit(f"{out} already exists; pick another --name")
    out.mkdir(parents=True)
    (out / "run.json").write_text(json.dumps(
        {"duration_s": args.duration, "eject_command_s": args.eject,
         "spin_hz": 0.0167, "eject_in_s": 5.0}, indent=2) + "\n")

    sim_cmd = ["gz", "sim", str(HERE / "picosat_deploy.sdf")]
    if args.headless:
        sim_cmd[2:2] = ["-s", "--headless-rendering"]
    sim_log = open(out / "gazebo.log", "w")
    # Own process group, so shutdown reaches the GUI client as well.
    sim = subprocess.Popen(sim_cmd, stdout=sim_log, stderr=subprocess.STDOUT,
                           start_new_session=True)
    recorders = []

    def stop_recorders():
        for proc in recorders:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        for proc in recorders:
            proc.wait()

    def stop_sim():
        if sim.poll() is None:
            os.killpg(sim.pid, signal.SIGTERM)
            sim.wait()

    def interrupted(*_):
        print("\nStopping...")
        stop_recorders()
        stop_sim()
        raise SystemExit(130)

    signal.signal(signal.SIGINT, interrupted)

    print("Starting Gazebo...")
    wait_for_topic("/picosat_b/camera/nz", sim)
    if not args.headless:
        wait_for_topic("/gui/camera/pose", sim)

    sim_time = [0.0]
    eject_now = threading.Event()

    def on_stats(msg):
        sim_time[0] = msg.sim_time.sec + msg.sim_time.nsec * 1e-9
        if sim_time[0] >= args.eject:
            eject_now.set()

    node = Node()
    node.subscribe(WorldStatistics, "/stats", on_stats)
    eject_pub = node.advertise("/eject", Empty)

    duration = str(args.duration)
    recorders.append(subprocess.Popen(
        [sys.executable, str(HERE / "log_picosats.py"),
         "-o", str(out / "motion.csv"), "-t", duration],
        stdout=subprocess.DEVNULL))
    recorders.append(subprocess.Popen(
        [sys.executable, str(HERE / "record_cameras.py"),
         "-o", str(out / "images"), "-t", duration],
        stdout=subprocess.DEVNULL))
    time.sleep(3)  # let the subscriptions connect before time starts

    subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/control",
                    "--reqtype", "gz.msgs.WorldControl",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
                    "--req", "pause: false"],
                   check=True, stdout=subprocess.DEVNULL)
    print(f"Running: eject at t = {args.eject:g} s, "
          f"stop at t = {args.duration:g} s")

    eject_now.wait()
    eject_pub.publish(Empty())
    print(f"Ejected at t = {sim_time[0]:.2f} s")

    # Generous limit in case the sim runs slower than real time.
    deadline = time.monotonic() + 10 * args.duration + 30
    while any(p.poll() is None for p in recorders):
        if sim.poll() is not None:
            print("Gazebo exited early; the recording is incomplete")
            break
        if time.monotonic() > deadline:
            print("Timed out; the recording may be incomplete")
            break
        time.sleep(0.2)
    stop_recorders()

    if args.keep_open:
        print("Leaving Gazebo open. Close its window when you're done.")
    else:
        stop_sim()
    print(f"Saved to {out}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Save the picosat camera streams from picosat_deploy.sdf as PNG files.

The cameras only render while something subscribes to them, so start this
before pressing play. Frames are named by sim time, which matches the t column
of log_picosats.py:

    images/picosat_a/px/t000012.300.png

    python3 record_cameras.py                   # until Ctrl-C -> images/
    python3 record_cameras.py -o run1 -t 30     # stop after 30 s of sim time
    python3 record_cameras.py --cameras a/px b/nz
"""

import argparse
import queue
import signal
import threading
from pathlib import Path

from gz.msgs.image_pb2 import Image as ImageMsg
from gz.transport import Node
from PIL import Image

SATS = ("a", "b")
FACES = ("px", "nx", "py", "ny", "pz", "nz")
ENCODERS = 6


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("-o", "--out", default="images")
    ap.add_argument("-t", "--duration", type=float, default=None,
                    help="stop after this many seconds of sim time")
    ap.add_argument("--cameras", nargs="+",
                    default=[f"{s}/{f}" for s in SATS for f in FACES],
                    help="cameras to record, as sat/face (default: all 12)")
    args = ap.parse_args()

    done = threading.Event()
    frames = queue.Queue()
    counts = {cam: 0 for cam in args.cameras}

    def encode():
        while True:
            item = frames.get()
            if item is None:
                return
            path, width, height, data = item
            Image.frombytes("RGB", (width, height), data).save(
                path, compress_level=1)

    workers = [threading.Thread(target=encode) for _ in range(ENCODERS)]
    for w in workers:
        w.start()

    def make_cb(cam, folder):
        def cb(msg):
            if done.is_set():
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            if args.duration is not None and stamp >= args.duration:
                done.set()
                return
            frames.put((folder / f"t{stamp:010.3f}.png",
                        msg.width, msg.height, msg.data))
            counts[cam] += 1
        return cb

    node = Node()
    for cam in args.cameras:
        sat, face = cam.split("/")
        folder = Path(args.out) / f"picosat_{sat}" / face
        folder.mkdir(parents=True, exist_ok=True)
        topic = f"/picosat_{sat}/camera/{face}"
        if not node.subscribe(ImageMsg, topic, make_cb(cam, folder)):
            raise SystemExit(f"could not subscribe to {topic}")

    signal.signal(signal.SIGINT, lambda *_: done.set())
    print(f"Recording {len(args.cameras)} cameras to {args.out}/ "
          "(Ctrl-C to stop). Press play in Gazebo if it is paused.")
    while not done.wait(0.2):
        pass

    for _ in workers:
        frames.put(None)
    for w in workers:
        w.join()
    print(f"Saved {sum(counts.values())} frames: "
          f"{min(counts.values())}-{max(counts.values())} per camera")


if __name__ == "__main__":
    main()

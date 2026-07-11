#!/usr/bin/env python
"""
dataset_publisher -- THE reusable image data source for the vision tutorial.

Publishes a sequence as ``sensor_msgs/Image`` on a topic via rclcppyy (the messages
are C++ ``sensor_msgs::msg::Image`` on the wire, so a rclcppyy subscriber gets them
with no Python copy). Sources, all behind one ``frame_source`` generator:
  * ``synthetic`` -- the deterministic synthetic loop (scripts/datasets/synthetic_loop.py),
    zero download; published as ``mono8``;
  * ``tum`` -- a TUM RGB-D sequence dir (uses its ``rgb/`` PNGs); published ``bgr8``;
  * ``folder`` -- any directory of images (sorted by filename); ``bgr8``.

Library::

    from dataset_publisher import frame_source, build_image_msg   # same-dir import
    for idx, frame, encoding in frame_source("synthetic"):
        ...

Standalone node::

    python scripts/datasets/dataset_publisher.py --synthetic --rate 10
    python scripts/datasets/dataset_publisher.py --tum data/rgbd_dataset_freiburg3_long_office_household
"""
import argparse
import glob
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)  # for same-dir synthetic_loop import
import synthetic_loop  # noqa: E402

TOPIC = "vision/image"


def _list_images(folder):
    exts = ("*.png", "*.jpg", "*.jpeg", "*.bmp")
    files = []
    for e in exts:
        files.extend(glob.glob(os.path.join(folder, e)))
    return sorted(files)


def frame_source(kind, path=None, n_frames=synthetic_loop.DEFAULT_N, gray=False, limit=None):
    """Yield ``(idx, frame_uint8, encoding)``. ``kind`` in {synthetic, tum, folder}.
    ``gray=True`` forces mono8 (e.g. to publish TUM as grayscale)."""
    if kind == "synthetic":
        for idx, frame in synthetic_loop.frames(n_frames):
            yield idx, frame, "mono8"
        return

    if kind == "tum":
        rgb = os.path.join(path, "rgb")
        files = _list_images(rgb if os.path.isdir(rgb) else path)
    elif kind == "folder":
        files = _list_images(path)
    else:
        raise ValueError("unknown source kind %r" % kind)
    if not files:
        raise SystemExit("no images found under %s" % path)
    import cv2
    for idx, f in enumerate(files):
        if limit is not None and idx >= limit:
            return
        if gray:
            img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
            yield idx, np.ascontiguousarray(img), "mono8"
        else:
            img = cv2.imread(f, cv2.IMREAD_COLOR)  # BGR
            yield idx, np.ascontiguousarray(img), "bgr8"


def build_image_msg(Image, frame, encoding, seq=0, frame_id="camera"):
    """Fill a C++ ``sensor_msgs::msg::Image`` from a uint8 frame. The pixel bytes
    are bulk-copied into the message's ``data`` vector in C++ (cppyy's
    ``std::vector<uint8_t>`` accepts the flat NumPy buffer directly)."""
    import cppyy
    h, w = frame.shape[:2]
    ch = 1 if frame.ndim == 2 else frame.shape[2]
    msg = Image()
    msg.height, msg.width = int(h), int(w)
    msg.encoding = encoding
    msg.step = int(w * ch)
    msg.is_bigendian = 0
    msg.data = cppyy.gbl.std.vector["uint8_t"](np.ascontiguousarray(frame).ravel())
    msg.header.frame_id = frame_id
    return msg


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--synthetic", action="store_true", help="publish the synthetic loop (default)")
    src.add_argument("--tum", metavar="DIR", help="publish a TUM RGB-D sequence dir")
    src.add_argument("--folder", metavar="DIR", help="publish an image folder")
    ap.add_argument("--rate", type=float, default=10.0, help="publish rate (Hz)")
    ap.add_argument("--topic", default=TOPIC)
    ap.add_argument("--n", type=int, default=synthetic_loop.DEFAULT_N, help="synthetic frame count")
    ap.add_argument("--limit", type=int, help="max frames (tum/folder)")
    ap.add_argument("--gray", action="store_true", help="publish mono8")
    ap.add_argument("--loop", action="store_true", help="republish forever")
    args = ap.parse_args()

    os.environ.setdefault("ROS_DOMAIN_ID", "50")
    from rclcppyy.bringup_rclcpp import bringup_rclcpp
    import cppyy
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    cppyy.include("sensor_msgs/msg/image.hpp")
    Image = cppyy.gbl.sensor_msgs.msg.Image

    if args.tum:
        kind, path = "tum", args.tum
    elif args.folder:
        kind, path = "folder", args.folder
    else:
        kind, path = "synthetic", None

    node = rclcpp.Node("dataset_publisher")
    pub = node.create_publisher(Image, args.topic, 10)
    print("Publishing %s -> %s at %.1f Hz" % (kind, args.topic, args.rate))

    period = 1.0 / args.rate
    published = 0
    while True:
        for idx, frame, enc in frame_source(kind, path, n_frames=args.n,
                                             gray=args.gray, limit=args.limit):
            msg = build_image_msg(Image, frame, enc, seq=idx)
            pub.publish(msg)
            published += 1
            if published % 20 == 0:
                print("  published %d (frame %d, %s %dx%d)"
                      % (published, idx, enc, frame.shape[1], frame.shape[0]), flush=True)
            time.sleep(period)
        if not args.loop:
            break
    print("Done: published %d frames." % published)


if __name__ == "__main__":
    main()

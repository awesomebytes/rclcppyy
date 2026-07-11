#!/usr/bin/env python
"""
train_vocab -- train a small DBoW2 ORB vocabulary on a sequence (the zero-download
alternative to the 145 MB ORBvoc), and save it.

ORB every frame of the source, then ``dbow_kit.train_vocabulary`` (kmeans++ tree,
seeded for reproducibility). Saves a ``.dbow2`` binary (fast to reload). Used to
generate the committed golden-test fixture and for anyone who wants a vocabulary
tuned to their own sequence without a big download.

    pixi run -e vision vocab-train                       # synthetic -> fixture path
    python scripts/vision/train_vocab.py --tum DIR --out data/tum_voc.dbow2 --k 10 --L 4
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "scripts", "datasets"))

import dataset_publisher as DP  # noqa: E402
from rclcppyy.kits import cv_kit, dbow_kit  # noqa: E402

FIXTURE = os.path.join(REPO, "test", "fixtures", "synthetic_voc.dbow2")


def train(kind="synthetic", path=None, k=10, L=4, nfeatures=1000, n_frames=200,
          limit=None, seed=0, gray=True):
    """ORB the source's frames and train a vocabulary; return it."""
    orb = cv_kit.create_orb(nfeatures)
    descs = []
    t0 = time.perf_counter()
    for _idx, frame, _enc in DP.frame_source(kind, path, n_frames=n_frames,
                                              gray=gray, limit=limit):
        _, desc = orb.detect_and_compute(cv_kit.numpy_to_mat(frame))
        if int(desc.rows) > 0:
            descs.append(desc)
    t_orb = time.perf_counter() - t0
    print("ORB %d frames in %.1f s" % (len(descs), t_orb))
    t0 = time.perf_counter()
    voc = dbow_kit.train_vocabulary(descs, k=k, L=L, seed=seed)
    print("trained vocab: %d words, k=%d L=%d, %.1f s" % (int(voc.size()), k, L,
                                                          time.perf_counter() - t0))
    return voc


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--tum", metavar="DIR")
    ap.add_argument("--folder", metavar="DIR")
    ap.add_argument("--out", default=FIXTURE, help="output .dbow2 path")
    ap.add_argument("--k", type=int, default=10)
    ap.add_argument("--L", type=int, default=4)
    ap.add_argument("--nfeatures", type=int, default=1000)
    ap.add_argument("--n", type=int, default=200, help="synthetic frame count")
    ap.add_argument("--limit", type=int, help="max frames (tum/folder)")
    args = ap.parse_args()

    kind = "tum" if args.tum else "folder" if args.folder else "synthetic"
    voc = train(kind, args.tum or args.folder, k=args.k, L=args.L,
                nfeatures=args.nfeatures, n_frames=args.n, limit=args.limit)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    dbow_kit.save_vocabulary(voc, args.out)
    print("saved %s (%.0f KB)" % (args.out, os.path.getsize(args.out) / 1024))


if __name__ == "__main__":
    main()

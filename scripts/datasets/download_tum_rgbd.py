#!/usr/bin/env python
"""
download_tum_rgbd -- fetch + verify a TUM RGB-D SLAM sequence with a REAL loop
closure into ``data/`` (gitignored), for the tutorial's real-data path.

Default: ``freiburg3_long_office_household`` -- the canonical loop-closure sequence
(the handheld camera circles a desk/office and returns to the start; used
throughout the ORB-SLAM papers). ~1.48 GB. Skips if already extracted; verifies the
download by byte size and successful extraction; prints the citation + license.

    python scripts/datasets/download_tum_rgbd.py                 # default sequence
    python scripts/datasets/download_tum_rgbd.py --list          # show choices
    python scripts/datasets/download_tum_rgbd.py --sequence freiburg1_room

Dataset: J. Sturm, N. Engelhard, F. Endres, W. Burgard, D. Cremers, "A Benchmark
for the Evaluation of RGB-D SLAM Systems", IROS 2012. Host: TUM CVG
(https://cvg.cit.tum.de/rgbd/dataset/). Data is CC BY 4.0 (attribute the authors).
"""
import argparse
import os
import sys
import tarfile
import time
import urllib.request

_BASE = "https://cvg.cit.tum.de/rgbd/dataset"

# name -> (freiburgN subdir, tgz basename, expected byte size for a size check).
SEQUENCES = {
    "freiburg3_long_office_household": (
        "freiburg3", "rgbd_dataset_freiburg3_long_office_household.tgz", 1483556251),
    "freiburg2_desk": (
        "freiburg2", "rgbd_dataset_freiburg2_desk.tgz", 1893351095),
    "freiburg1_room": (
        "freiburg1", "rgbd_dataset_freiburg1_room.tgz", 782381450),
}
DEFAULT = "freiburg3_long_office_household"

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DATA = os.path.join(REPO, "data")


def _url(seq):
    sub, base, _ = SEQUENCES[seq]
    return "%s/%s/%s" % (_BASE, sub, base)


def _report(count, block, total):
    done = count * block
    pct = 100.0 * done / total if total > 0 else 0.0
    sys.stdout.write("\r  %.0f%% (%.0f/%.0f MB)" % (pct, done / 1e6, total / 1e6))
    sys.stdout.flush()


def download(seq, force=False):
    _sub, base, expected = SEQUENCES[seq]
    os.makedirs(DATA, exist_ok=True)
    extracted = os.path.join(DATA, base[:-4])  # strip .tgz
    if os.path.isdir(extracted) and not force:
        print("Already present: %s (skip)" % extracted)
        return extracted
    tgz = os.path.join(DATA, base)
    url = _url(seq)
    print("Downloading %s\n  %s\n  -> %s (~%.0f MB)" % (seq, url, tgz, expected / 1e6))
    t0 = time.time()
    urllib.request.urlretrieve(url, tgz, _report)
    dt = time.time() - t0
    size = os.path.getsize(tgz)
    print("\n  downloaded %.0f MB in %.0f s (%.1f MB/s)" % (size / 1e6, dt, size / 1e6 / dt))
    if abs(size - expected) > 1_000_000:
        sys.exit("ERROR: size %d != expected %d (corrupt/incomplete download)" % (size, expected))
    print("  size verified. Extracting ...")
    with tarfile.open(tgz) as tf:
        tf.extractall(DATA)
    os.remove(tgz)
    print("Extracted to %s" % extracted)
    return extracted


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sequence", default=DEFAULT, choices=sorted(SEQUENCES))
    ap.add_argument("--list", action="store_true", help="list sequences and exit")
    ap.add_argument("--force", action="store_true", help="re-download even if present")
    args = ap.parse_args()
    if args.list:
        for name, (_s, _b, sz) in sorted(SEQUENCES.items()):
            print("  %-34s ~%.0f MB%s" % (name, sz / 1e6, "  (default)" if name == DEFAULT else ""))
        return
    path = download(args.sequence, force=args.force)
    print("\nCitation: Sturm et al., 'A Benchmark for the Evaluation of RGB-D SLAM "
          "Systems', IROS 2012 (TUM CVG). License: CC BY 4.0.")
    print("Publish it as sensor_msgs/Image with:")
    print("  python scripts/datasets/dataset_publisher.py --tum %s" % path)


if __name__ == "__main__":
    main()

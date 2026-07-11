#!/usr/bin/env python
"""
download_orbvoc -- fetch the real ORB vocabulary (ORBvoc.txt, ~145 MB) into
``data/`` (gitignored), for the tutorial's real place-recognition path.

This is the vocabulary shipped with ORB-SLAM2 (Galvez-Lopez & Tardos DBoW2, trained
on a large image corpus): a k=10, L=6, ~1M-word tree. ~42.5 MB compressed. Skips if
already present; verifies by extraction.

Parsing the 145 MB text is slow (tens of seconds), so dbow_kit caches a binary
``ORBvoc.txt.dbow2`` next to it on first load and reuses it (~1 s) thereafter --
handled automatically by ``dbow_kit.load_vocabulary``; nothing to do here.

    python scripts/datasets/download_orbvoc.py
    pixi run -e vision demo-vision-loop --tum data/<seq> --vocab data/ORBvoc.txt
"""
import os
import sys
import tarfile
import time
import urllib.request

URL = "https://raw.githubusercontent.com/raulmur/ORB_SLAM2/master/Vocabulary/ORBvoc.txt.tar.gz"
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DATA = os.path.join(REPO, "data")
TXT = os.path.join(DATA, "ORBvoc.txt")


def _report(count, block, total):
    if total <= 0:
        return
    pct = int(100.0 * count * block / total)
    if pct != getattr(_report, "_p", -1):
        _report._p = pct
        end = "\r" if sys.stdout.isatty() else "\n"
        sys.stdout.write("%s  %d%% (%.0f/%.0f MB)" % (end, pct, count * block / 1e6, total / 1e6))
        sys.stdout.flush()


def download():
    if os.path.isfile(TXT):
        print("Already present: %s (%.0f MB)" % (TXT, os.path.getsize(TXT) / 1e6))
        return TXT
    os.makedirs(DATA, exist_ok=True)
    tgz = os.path.join(DATA, "ORBvoc.txt.tar.gz")
    print("Downloading ORBvoc (ORB-SLAM2 DBoW2 vocabulary)\n  %s" % URL)
    t0 = time.time()
    urllib.request.urlretrieve(URL, tgz, _report)
    print("\n  downloaded %.0f MB in %.0f s. Extracting ..."
          % (os.path.getsize(tgz) / 1e6, time.time() - t0))
    with tarfile.open(tgz) as tf:
        tf.extractall(DATA)
    os.remove(tgz)
    if not os.path.isfile(TXT):
        sys.exit("ERROR: ORBvoc.txt not found after extraction")
    print("Extracted %s (%.0f MB)" % (TXT, os.path.getsize(TXT) / 1e6))
    return TXT


def main():
    download()
    print("\nVocabulary: Galvez-Lopez & Tardos, 'Bags of Binary Words for Fast Place "
          "Recognition in Image Sequences', IEEE T-RO 2012 (via ORB-SLAM2).")
    print("Use it: --vocab data/ORBvoc.txt (first load parses ~145 MB text, then "
          "caches a binary alongside for ~1 s reloads).")


if __name__ == "__main__":
    main()

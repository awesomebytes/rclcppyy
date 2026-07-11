#!/usr/bin/env python
"""
synthetic_loop -- a DETERMINISTIC procedural image sequence with a GUARANTEED
revisit, for testing / CI / instant-start of the visual loop-closure tutorial with
ZERO download.

A large fixed-seed textured canvas (scattered high-contrast rectangles on mid-gray
-- rich in the axis-aligned corners ORB latches onto, and stable under small
translation) is viewed through a sliding window that travels a closed rectangular
circuit and RETURNS to the start: the last ``revisit`` frames retrace the positions
of the first ``revisit`` frames, so frames near the end see the same scene as frames
near the beginning -- a loop closure by construction. Seeded throughout, so every
run yields identical frames (and thus an identical, recordable loop-detection
baseline -- the golden test's contract).

Library (import from the scripts/datasets dir -- it is not a package)::

    import sys; sys.path.insert(0, "scripts/datasets")
    import synthetic_loop as S
    for idx, frame in S.frames(n_frames=200):     # frame: (480,640) uint8
        ...
    S.loop_segment(200)        # (start, end) frame range that revisits the opening

CLI (optional; writes PNGs -- needs cv2)::

    python scripts/datasets/synthetic_loop.py --out data/synthetic_loop --n 200
"""
import argparse
import os

import numpy as np

WIN_H, WIN_W = 480, 640
CANVAS = 1400
DEFAULT_N = 200
DEFAULT_REVISIT = 20
SEED = 20260711


def canvas(seed=SEED, size=CANVAS, n_shapes=900):
    """Build the fixed textured canvas (mid-gray + scattered bright/dark
    rectangles + mild low-frequency shading). Pure NumPy, fully deterministic."""
    rng = np.random.default_rng(seed)
    img = np.full((size, size), 128, dtype=np.uint8)
    # Low-frequency shading so large flat regions still differ slightly.
    yy, xx = np.mgrid[0:size, 0:size]
    shade = (16 * np.sin(xx / 190.0) + 16 * np.cos(yy / 210.0)).astype(np.int16)
    img = np.clip(img.astype(np.int16) + shade, 0, 255).astype(np.uint8)
    # Scattered rectangles: strong, translation-stable corners.
    for _ in range(n_shapes):
        h = int(rng.integers(6, 40))
        w = int(rng.integers(6, 40))
        y = int(rng.integers(0, size - h))
        x = int(rng.integers(0, size - w))
        val = int(rng.integers(0, 256))
        img[y:y + h, x:x + w] = val
    return img


def _lap_positions(n, canvas_size=CANVAS):
    """``n`` (top,left) window positions evenly spaced along one loop of the
    rectangular circuit (clockwise from the top-left corner)."""
    tmax = canvas_size - WIN_H
    lmax = canvas_size - WIN_W
    # Four edges of the circuit as (top,left) endpoints, closed back to start.
    corners = [(0, 0), (0, lmax), (tmax, lmax), (tmax, 0), (0, 0)]
    # Cumulative length for even spacing.
    seglen = [abs(corners[i + 1][0] - corners[i][0]) + abs(corners[i + 1][1] - corners[i][1])
              for i in range(4)]
    total = float(sum(seglen))
    out = []
    for i in range(n):
        d = total * i / n
        e = 0
        while e < 3 and d > seglen[e]:
            d -= seglen[e]
            e += 1
        (t0, l0), (t1, l1) = corners[e], corners[e + 1]
        f = d / seglen[e] if seglen[e] else 0.0
        out.append((int(round(t0 + (t1 - t0) * f)), int(round(l0 + (l1 - l0) * f))))
    return out


def positions(n_frames=DEFAULT_N, revisit=DEFAULT_REVISIT, canvas_size=CANVAS):
    """The full trajectory: ``n_frames - revisit`` positions around one lap, then
    ``revisit`` positions that EXACTLY retrace the first ``revisit`` positions
    (the guaranteed loop closure)."""
    if n_frames <= revisit + 3:
        raise ValueError("n_frames (%d) must exceed revisit (%d) by more than 3 "
                         "so there is a lap to close" % (n_frames, revisit))
    base = _lap_positions(n_frames - revisit, canvas_size)
    return base + base[:revisit]


def loop_segment(n_frames=DEFAULT_N, revisit=DEFAULT_REVISIT):
    """``(start, end)`` half-open frame range whose frames revisit the opening --
    i.e. frame ``start+j`` reprises the scene of frame ``j``."""
    return (n_frames - revisit, n_frames)


def frames(n_frames=DEFAULT_N, revisit=DEFAULT_REVISIT, seed=SEED, noise=8, canvas_size=CANVAS):
    """Yield ``(idx, frame)`` for the whole sequence. Each ``frame`` is a
    ``(480,640)`` uint8 window crop plus mild per-frame deterministic noise (so a
    revisit is realistically *similar*, not byte-identical, yet its structure
    dominates -> a strong, unambiguous loop). Fully reproducible."""
    cv = canvas(seed=seed, size=canvas_size)
    pos = positions(n_frames, revisit, canvas_size)
    for idx, (t, l) in enumerate(pos):
        crop = cv[t:t + WIN_H, l:l + WIN_W].astype(np.int16)
        if noise:
            nrng = np.random.default_rng(seed + 1000 + idx)
            crop = crop + nrng.integers(-noise, noise + 1, crop.shape, dtype=np.int16)
        yield idx, np.clip(crop, 0, 255).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", help="directory to write frameNNNNNN.png into")
    ap.add_argument("--n", type=int, default=DEFAULT_N, help="number of frames")
    ap.add_argument("--revisit", type=int, default=DEFAULT_REVISIT,
                    help="length of the revisit (loop-closure) segment")
    args = ap.parse_args()

    s, e = loop_segment(args.n, args.revisit)
    print("synthetic loop: %d frames, %dx%d; loop segment frames [%d,%d) revisit [0,%d)"
          % (args.n, WIN_W, WIN_H, s, e, args.revisit))
    if not args.out:
        # Just materialize the generator (sanity: no crash, right shapes).
        shapes = {f.shape for _, f in frames(args.n, args.revisit)}
        print("frame shapes:", shapes, "(use --out DIR to write PNGs)")
        return
    try:
        import cv2
    except ImportError:
        raise SystemExit("--out needs cv2 (pip/conda opencv). Omit it to just yield arrays.")
    os.makedirs(args.out, exist_ok=True)
    for idx, frame in frames(args.n, args.revisit):
        cv2.imwrite(os.path.join(args.out, "frame%06d.png" % idx), frame)
    print("wrote %d PNGs to %s" % (args.n, args.out))


if __name__ == "__main__":
    main()

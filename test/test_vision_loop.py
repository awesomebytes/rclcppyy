#!/usr/bin/env python3
"""GOLDEN TEST for the visual loop-closure front-end (the ATO contract).

On the DETERMINISTIC synthetic loop sequence (scripts/datasets/synthetic_loop.py --
seeded, byte-identical every run, ZERO download), run the full front-end -- C++ ORB
-> DBoW2 vocabulary (trained in-process, kmeans++ seeded) -> OrbDatabase query ->
temporal-consistency gate (loop_detector.py) -- and assert the detected loop-pair
set matches a recorded baseline (tolerant matching). This is the regression contract
for the whole pipeline: if ORB, the DBoW2 build, the marshaling, or the detection
logic drifts, the detected pairs change and this fails.

Auto-skips when the vision env / vendored DBoW2 is absent, so the default
`pixi run test` is unaffected. Run with `pixi run -e vision test-vision`.

NO download: the sequence and the vocabulary are both generated on the fly.
"""
import glob
import os
import sys

import pytest

_CONDA = os.environ.get("CONDA_PREFIX", "")
_HAVE_CV = bool(glob.glob(os.path.join(_CONDA, "include", "opencv4")))
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)
_HAVE_DBOW = os.path.isfile(os.path.join(_REPO, "build", "vendor", "libDBoW2.so"))

pytestmark = pytest.mark.skipif(
    not (_HAVE_CV and _HAVE_DBOW),
    reason="vision env / vendored DBoW2 not present (use the vision env + build-dbow2)")

# Recorded baseline: on the 200-frame synthetic sequence (revisit segment [180,200)
# reprising frames [0,20)), the temporally-consistent detector confirms frame
# 180+j revisiting frame j for j=1..19 (frame 180 itself is inside the k=3
# confirmation warm-up). Deterministic within the pinned OpenCV/DBoW2 build.
N_FRAMES = 200
BASELINE = [(180 + j, j) for j in range(1, 20)]   # (query, match)
MIN_SCORE, CONSISTENCY_K, IGNORE_RECENT = 0.4, 3, 25

if _HAVE_CV and _HAVE_DBOW:
    sys.path.insert(0, os.path.join(_REPO, "scripts", "datasets"))
    sys.path.insert(0, os.path.join(_REPO, "scripts", "vision"))
    import synthetic_loop  # noqa: E402
    from loop_detector import LoopDetector  # noqa: E402
    from rclcppyy.kits import cv_kit, dbow_kit  # noqa: E402


@pytest.fixture(scope="module")
def descriptors():
    """ORB every synthetic frame once (reused for training and detection)."""
    orb = cv_kit.create_orb(1000)
    return [orb.detect_and_compute(cv_kit.numpy_to_mat(f))[1]
            for _, f in synthetic_loop.frames(N_FRAMES)]


def _detect(descs):
    voc = dbow_kit.train_vocabulary(descs, k=10, L=4, seed=0)
    det = LoopDetector(voc, min_score=MIN_SCORE, consistency_k=CONSISTENCY_K,
                       ignore_recent=IGNORE_RECENT)
    out = []
    for d in descs:
        lc = det.add_and_query(d)
        if lc:
            out.append((lc.query_id, lc.match_id))
    return out


def _pairs_match(detected, baseline, qtol=1, mtol=3):
    """Tolerant set match: every baseline pair has a detected pair within
    (query +-qtol, match +-mtol), and there are no spurious detections outside
    that tolerance of some baseline pair."""
    def near(a, b):
        return abs(a[0] - b[0]) <= qtol and abs(a[1] - b[1]) <= mtol
    covered = all(any(near(d, b) for d in detected) for b in baseline)
    no_spurious = all(any(near(d, b) for b in baseline) for d in detected)
    return covered and no_spurious


def test_golden_loop_pairs(descriptors):
    """The detected loop-pair set matches the recorded baseline (tolerant)."""
    detected = _detect(descriptors)
    assert detected, "no loops detected on the synthetic sequence"
    assert _pairs_match(detected, BASELINE), \
        "detected pairs %s do not match baseline %s" % (detected, BASELINE)


def test_precision_is_one_by_construction(descriptors):
    """Every confirmed loop is a TRUE revisit (query in the revisit segment,
    matching frame == query - segment_start). Synthetic precision = 1.0."""
    seg_start, seg_end = synthetic_loop.loop_segment(N_FRAMES)
    detected = _detect(descriptors)
    for q, m in detected:
        assert seg_start <= q < seg_end, "detection %d outside revisit segment" % q
        assert abs(m - (q - seg_start)) <= 3, "detection (%d,%d) is not a true revisit" % (q, m)


def test_training_is_deterministic(descriptors):
    """Seeded vocabulary training -> identical detections run to run (the property
    the golden baseline relies on)."""
    assert _detect(descriptors) == _detect(descriptors)


def test_no_false_loops_before_revisit(descriptors):
    """No loop is confirmed before the camera nears the start again (the whole
    first lap has no true revisit)."""
    detected = _detect(descriptors)
    seg_start, _ = synthetic_loop.loop_segment(N_FRAMES)
    early = [q for q, _ in detected if q < seg_start - IGNORE_RECENT]
    assert not early, "false loops confirmed mid-lap: %s" % early

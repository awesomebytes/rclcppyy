#!/usr/bin/env python3
"""Tests for rclcppyy.kits.cv_kit + dbow_kit (OpenCV + DBoW2 via cppyy).

These are optional dependencies (the pixi `vision` env, plus the vendored DBoW2
built by `pixi run -e vision build-dbow2`). The whole module auto-skips when the
OpenCV headers or libDBoW2.so are absent, so the default `pixi run test` is
unaffected. Run the real thing with `pixi run -e vision test-vision`.

Covered here (M0-M2 foundations): bringup idempotency, the CUDA auto-detect clean
absence, the **zero-copy** buffer bridge (numpy/Mat and sensor_msgs/Image -> Mat
pointer identity -- the headline evidence), ORB descriptor shape (Nx32 CV_8U), and
the DBoW2 train / binary-cache / query path. The temporal-consistency loop
detection and its golden baseline live in test_vision_loop.py.
"""
import glob
import os

import numpy as np
import pytest

_CONDA = os.environ.get("CONDA_PREFIX", "")
_HAVE_CV = bool(glob.glob(os.path.join(_CONDA, "include", "opencv4")))
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)
_HAVE_DBOW = os.path.isfile(os.path.join(_REPO, "build", "vendor", "libDBoW2.so"))

pytestmark = pytest.mark.skipif(
    not (_HAVE_CV and _HAVE_DBOW),
    reason="vision env / vendored DBoW2 not present (use the vision env + build-dbow2)")

if _HAVE_CV and _HAVE_DBOW:
    from rclcppyy.kits import cv_kit, dbow_kit


@pytest.fixture(scope="module")
def cv():
    return cv_kit.bringup_cv()


@pytest.fixture(scope="module")
def orb():
    return cv_kit.create_orb(500)


def _synthetic_frame(seed):
    """A textured gray frame ORB finds plenty of corners in."""
    rng = np.random.default_rng(seed)
    return rng.integers(0, 256, (240, 320), dtype=np.uint8)


# --- cv_kit -------------------------------------------------------------------

def test_bringup_idempotent(cv):
    assert cv is cv_kit.bringup_cv()
    assert hasattr(cv, "ORB")


def test_cuda_absent_is_clean(cv):
    # conda-forge OpenCV has no CUDA build; this must return False, not raise.
    assert cv_kit.cuda_available() is False


def test_numpy_to_mat_zero_copy(cv):
    img = _synthetic_frame(0)
    mat = cv_kit.numpy_to_mat(img)
    glue = cv_kit._glue()
    assert int(mat.rows) == 240 and int(mat.cols) == 320
    # The Mat aliases the NumPy buffer: identical data pointer, no copy.
    assert int(glue.mat_data_addr(mat)) == img.ctypes.data


def test_mat_to_numpy_roundtrip(cv):
    img = _synthetic_frame(1)
    mat = cv_kit.numpy_to_mat(img)
    back = cv_kit.mat_to_numpy(mat, copy=True)
    assert back.shape == (240, 320)
    assert np.array_equal(back, img)


def test_orb_descriptor_shape(orb):
    img = _synthetic_frame(2)
    kps, desc = orb.detect_and_compute(cv_kit.numpy_to_mat(img))
    assert int(kps.size()) > 100
    # ORB descriptors: N x 32, 8-bit (256-bit binary descriptor).
    assert int(desc.rows) == int(kps.size())
    assert int(desc.cols) == 32
    d = cv_kit.descriptors_to_numpy(desc)
    assert d.shape == (int(kps.size()), 32) and d.dtype == np.uint8
    xy = cv_kit.keypoints_to_numpy(kps)
    assert xy.shape == (int(kps.size()), 2) and xy.dtype == np.float32


def test_msg_to_mat_zero_copy_pointer_identity(cv):
    """The headline zero-copy evidence: a C++ sensor_msgs/Image's data buffer and
    the wrapping cv::Mat share ONE pointer (no per-frame copy into Python)."""
    import cppyy
    from rclcppyy.bringup_rclcpp import add_ros2_include_paths
    add_ros2_include_paths()
    cppyy.include("sensor_msgs/msg/image.hpp")
    Image = cppyy.gbl.sensor_msgs.msg.Image
    msg = Image()
    msg.width, msg.height, msg.encoding, msg.step = 320, 240, "mono8", 320
    msg.data.resize(320 * 240)
    mat = cv_kit.msg_to_mat(msg)
    glue = cv_kit._glue()
    assert int(mat.rows) == 240 and int(mat.cols) == 320
    assert int(glue.mat_data_addr(mat)) == int(glue.vec_data_addr(msg.data))
    # Aliasing is real: writing through the message is visible in the Mat.
    msg.data[100 * 320 + 50] = 222
    assert cv_kit.mat_to_numpy(mat, copy=False)[100, 50] == 222


# --- dbow_kit -----------------------------------------------------------------

@pytest.fixture(scope="module")
def descriptor_set(orb):
    """ORB descriptors for a handful of distinct synthetic frames."""
    mats = []
    for i in range(8):
        _, desc = orb.detect_and_compute(cv_kit.numpy_to_mat(_synthetic_frame(10 + i)))
        mats.append(desc)
    return mats


def test_train_vocabulary(descriptor_set):
    voc = dbow_kit.train_vocabulary(descriptor_set, k=9, L=3)
    assert int(voc.size()) > 0


def test_database_self_match(descriptor_set):
    voc = dbow_kit.train_vocabulary(descriptor_set, k=9, L=3)
    db = dbow_kit.make_database(voc)
    for desc in descriptor_set:
        dbow_kit.add_image(db, desc)
    # Querying with frame 3's own descriptors must rank frame 3 first at score ~1.
    results = dbow_kit.query(db, descriptor_set[3], max_results=3)
    assert results[0][0] == 3
    assert results[0][1] > 0.99


def test_binary_cache_roundtrip(descriptor_set, tmp_path):
    voc = dbow_kit.train_vocabulary(descriptor_set, k=9, L=3)
    path = str(tmp_path / "voc.dbow2")
    dbow_kit.save_vocabulary(voc, path)
    voc2 = dbow_kit.load_vocabulary(path)
    assert int(voc2.size()) == int(voc.size())

    def ranking(v):
        db = dbow_kit.make_database(v)
        for desc in descriptor_set:
            dbow_kit.add_image(db, desc)
        return [i for i, _ in dbow_kit.query(db, descriptor_set[2], max_results=4)]

    assert ranking(voc) == ranking(voc2)


def test_query_max_id_excludes_current(descriptor_set):
    voc = dbow_kit.train_vocabulary(descriptor_set, k=9, L=3)
    db = dbow_kit.make_database(voc)
    for desc in descriptor_set:
        dbow_kit.add_image(db, desc)
    # max_id is inclusive: querying frame 5 with max_id=2 must return only id<=2.
    results = dbow_kit.query(db, descriptor_set[5], max_results=3, max_id=2)
    assert all(i <= 2 for i, _ in results)

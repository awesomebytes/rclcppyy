"""
cv_kit -- drive OpenCV's C++ API (core / imgproc / features2d) from Python via
cppyy, with a **zero-copy** bridge to ROS 2 ``sensor_msgs/Image`` messages.

OpenCV ships a mature Python binding (``cv2``), so unlike pcl_kit this kit is not
about "impossible -> possible". Its reason to exist is **composition**: it lets an
rclcppyy subscription hand its **C++** ``sensor_msgs::msg::Image`` straight into
``cv::Mat`` with no copy (the message's ``data`` buffer *is* the Mat's storage),
run C++ ``cv::ORB`` on it, and pass the resulting ``cv::Mat`` descriptors on to
DBoW2 (dbow_kit) -- the whole vision front-end stays in one C++ address space,
with Python only orchestrating. cv2 would force a serialize/copy at every hop.

The kit mirrors OpenCV's own API (``cv.ORB.create()``, ``detectAndCompute``,
``cv.cvtColor`` ...) on the returned ``cv`` namespace and adds only the glue cppyy
needs:
  * bringup -- add the ``include/opencv4`` path, JIT-include core/imgproc(/features2d),
    ``load_library`` the ``libopencv_*.so`` set so symbols resolve at call time;
  * the Mat<->buffer bridge -- ``cv::Mat`` cannot be constructed from a raw Python
    integer address (cppyy rejects the ``void*`` argument), and OpenCV's type
    constants (``CV_8UC1`` ...) are C **macros** invisible to cppyy, so both live in
    a tiny ``cppdef`` helper addressed via ``uintptr_t``;
  * ``msg_to_mat`` -- wrap a C++ ``sensor_msgs::msg::Image``'s ``data`` vector as a
    ``cv::Mat`` with **no copy** (pointer-identical to ``msg.data.data()``). The Mat
    aliases the message's storage, so it must not outlive the message (see below);
  * CUDA auto-detect -- ``cuda_available()`` probes for the ``cudafeatures2d`` module
    at bringup; absent (the conda-forge build has no CUDA), the CPU ``cv::ORB`` path
    is used with a one-time notice. A CUDA-enabled ``libopencv`` drops in with no
    code change: ``create_orb`` switches on the single ``use_cuda`` branch point.

Lifetime rule (the "dangling Mat" footgun): ``msg_to_mat`` and
``mat_to_numpy(copy=False)`` return views that ALIAS C++ / message storage. Keep
the backing object (the ``Image`` message, or the ``Mat``) alive for as long as you
use the view. ``msg_to_mat`` pins the message on the Mat best-effort, but the
contract is "use the Mat within the callback that owns the message".
"""
import ctypes
import os

import cppyy

from rclcppyy.kits import cppyy_kit

_CV_LIBS = (
    "libopencv_core.so",
    "libopencv_imgproc.so",
    "libopencv_features2d.so",
)

# Core + features2d headers. imgproc gives us cvtColor (color -> gray for ORB).
_CV_HEADERS = ("opencv2/core.hpp", "opencv2/imgproc.hpp", "opencv2/features2d.hpp")

# C++ glue compiled once at bringup. Two things force this into C++:
#   1. cv::Mat's (rows, cols, type, void* data, step) constructor cannot be called
#      from Python -- cppyy will not convert a Python int to the void* data arg. So
#      Mat-from-buffer must be a C++ helper taking the address as uintptr_t.
#   2. CV_8UC1 / CV_8U / ... are C preprocessor macros, invisible to cppyy. We
#      re-expose the handful we need as real constants.
# The keypoint/descriptor extractors also live here so the per-frame marshaling is
# a single C++ memcpy/loop rather than a Python per-element crossing.
_CPP_GLUE = r"""
namespace rclcppyy_cvkit {
  const int CV_8U_    = CV_8U;
  const int CV_8UC1_  = CV_8UC1;
  const int CV_8UC3_  = CV_8UC3;
  const int CV_8UC4_  = CV_8UC4;
  const int CV_16UC1_ = CV_16UC1;

  // Address of a message's byte buffer (sensor_msgs::msg::Image::data is a
  // std::vector<uint8_t>); used to alias it as a Mat with no copy.
  inline uintptr_t vec_data_addr(const std::vector<uint8_t>& v) {
    return reinterpret_cast<uintptr_t>(v.data());
  }
  inline size_t vec_size(const std::vector<uint8_t>& v) { return v.size(); }

  // Wrap an existing byte buffer as a Mat of the given cv type WITHOUT copying.
  inline cv::Mat mat_from_buffer(int rows, int cols, int type, uintptr_t data, size_t step) {
    return cv::Mat(rows, cols, type, reinterpret_cast<void*>(data), step);
  }
  inline uintptr_t mat_data_addr(const cv::Mat& m) {
    return reinterpret_cast<uintptr_t>(m.data);
  }
  inline size_t mat_step(const cv::Mat& m) { return m.step; }

  // Copy an Nx32 CV_8U descriptor Mat into a caller-owned (N*32) byte buffer.
  inline void copy_u8_mat(const cv::Mat& m, uintptr_t dst) {
    cv::Mat mc = m.isContinuous() ? m : m.clone();
    std::memcpy(reinterpret_cast<void*>(dst), mc.data,
                static_cast<size_t>(mc.rows) * mc.cols * mc.elemSize());
  }
  // Copy keypoint (x,y) pairs into a caller-owned (N*2) float buffer.
  inline void keypoints_xy(const std::vector<cv::KeyPoint>& kps, uintptr_t dst) {
    float* d = reinterpret_cast<float*>(dst);
    for (size_t i = 0; i < kps.size(); ++i) { d[2*i] = kps[i].pt.x; d[2*i+1] = kps[i].pt.y; }
  }
}
"""

# encoding -> (glue constant name, bytes-per-pixel). Covers the encodings a mono/
# color camera or the dataset publisher emit. rgb8/bgr8 differ only in channel
# order (irrelevant to the Mat wrap; convert with cvtColor if you care).
_ENCODING = {
    "mono8": ("CV_8UC1_", 1), "8UC1": ("CV_8UC1_", 1),
    "bgr8": ("CV_8UC3_", 3), "rgb8": ("CV_8UC3_", 3), "8UC3": ("CV_8UC3_", 3),
    "bgra8": ("CV_8UC4_", 4), "rgba8": ("CV_8UC4_", 4),
    "mono16": ("CV_16UC1_", 2), "16UC1": ("CV_16UC1_", 2),
}

_CV = None
_DONE = False
_CUDA = None  # tri-state: None = not yet probed


def _conda():
    return os.environ["CONDA_PREFIX"]


def bringup_cv():
    """Bring up OpenCV under cppyy and return the ``cv`` namespace. Idempotent.

    Adds ``$CONDA_PREFIX/include/opencv4``, JIT-includes core/imgproc/features2d,
    loads the ``libopencv_*.so`` set so calls resolve without ``LD_LIBRARY_PATH``,
    and compiles the Mat<->buffer glue. Use OpenCV's own API on the returned
    namespace directly (``cv.ORB.create()``, ``cv.cvtColor``, ...).
    """
    global _CV, _DONE
    if _DONE:
        return _CV
    conda = _conda()
    inc = os.path.join(conda, "include", "opencv4")
    if not os.path.isdir(inc):
        raise RuntimeError(
            "OpenCV headers not found at %s. Install the vision env: "
            "pixi install -e vision" % inc)
    cppyy.add_include_path(inc)
    for header in _CV_HEADERS:
        cppyy.include(header)
    cppyy_kit.load_libraries(_CV_LIBS, [os.path.join(conda, "lib")])
    cppyy.cppdef(_CPP_GLUE)
    _CV = cppyy.gbl.cv
    _DONE = True
    return _CV


def cuda_available():
    """Whether this OpenCV build has the CUDA features2d module (``cv::cuda::ORB``).

    Auto-detected once: the conda-forge OpenCV has **no** CUDA build, so this
    returns False here and the CPU ``cv::ORB`` path is used. A user-supplied
    CUDA-enabled ``libopencv`` (e.g. JetPack, or a self-built OpenCV with
    ``-DWITH_CUDA``) makes it True with no other change to this kit. Detection is
    by header presence (``opencv2/cudafeatures2d.hpp``) plus a live
    ``cv::cuda::getCudaEnabledDeviceCount() > 0`` check.
    """
    global _CUDA
    if _CUDA is not None:
        return _CUDA
    header = os.path.join(_conda(), "include", "opencv4", "opencv2", "cudafeatures2d.hpp")
    if not os.path.isfile(header):
        _CUDA = False
        return _CUDA
    try:
        bringup_cv()
        cppyy.include("opencv2/core/cuda.hpp")
        cppyy.include("opencv2/cudafeatures2d.hpp")
        cppyy_kit.load_libraries(("libopencv_cudafeatures2d.so",), [os.path.join(_conda(), "lib")])
        _CUDA = int(cppyy.gbl.cv.cuda.getCudaEnabledDeviceCount()) > 0
    except Exception:
        _CUDA = False
    return _CUDA


def _glue():
    bringup_cv()
    return cppyy.gbl.rclcppyy_cvkit


def msg_to_mat(image_msg):
    """Wrap a C++ ``sensor_msgs::msg::Image``'s ``data`` buffer as a ``cv::Mat``
    with **NO copy**.

    The returned Mat's storage *is* the message's ``data`` vector (pointer-identical
    to ``msg.data.data()``), so this is genuinely zero-copy -- the alternative
    (rclpy delivering ``msg.data`` as bytes, then ``np.frombuffer(...).reshape(...)``)
    copies the whole frame into Python on every message. The Mat aliases the
    message; **keep the message alive while you use the Mat** (this is pinned
    best-effort, but the contract is "use it within the owning callback").
    Unknown encodings raise. Returns the Mat.
    """
    bringup_cv()
    glue = _glue()
    enc = str(image_msg.encoding)
    if enc not in _ENCODING:
        raise ValueError(
            "cv_kit.msg_to_mat: unsupported encoding %r (known: %s)"
            % (enc, ", ".join(sorted(_ENCODING))))
    const_name, _bpp = _ENCODING[enc]
    type_int = int(getattr(glue, const_name))
    addr = int(glue.vec_data_addr(image_msg.data))
    step = int(image_msg.step)
    mat = glue.mat_from_buffer(int(image_msg.height), int(image_msg.width),
                               type_int, addr, step)
    # Pin the message on the Mat so the buffer cannot be freed while the view is
    # used (best-effort: a cppyy proxy may reject the attribute).
    cppyy_kit.keep_alive(mat, image_msg)
    return mat


def numpy_to_mat(array):
    """Wrap a contiguous 8-bit NumPy image (``(H,W)`` gray or ``(H,W,3)`` color) as
    a ``cv::Mat`` with **no copy**. The Mat aliases the NumPy buffer -- keep the
    array alive while you use the Mat. Used to feed synthetic frames to the C++ ORB
    without a round-trip through a ROS message."""
    import numpy as np
    arr = np.ascontiguousarray(array)
    if arr.dtype != np.uint8:
        raise ValueError("numpy_to_mat expects uint8, got %s" % arr.dtype)
    glue = _glue()
    if arr.ndim == 2:
        h, w = arr.shape
        type_int = int(glue.CV_8UC1_)
        step = w
    elif arr.ndim == 3 and arr.shape[2] == 3:
        h, w = arr.shape[:2]
        type_int = int(glue.CV_8UC3_)
        step = w * 3
    else:
        raise ValueError("numpy_to_mat expects (H,W) or (H,W,3) uint8, got %s" % (arr.shape,))
    mat = glue.mat_from_buffer(h, w, type_int, arr.ctypes.data, step)
    cppyy_kit.keep_alive(mat, arr)
    return mat


def mat_to_numpy(mat, copy=True):
    """Extract a ``cv::Mat`` (8-bit, 1 or 3 channel) to a NumPy array.

    ``copy=True`` (default) is a private copy, safe after the Mat is gone.
    ``copy=False`` is a zero-copy view that **aliases** the Mat's storage (respecting
    its row ``step``) -- keep the Mat (and any message it aliases) alive while the
    view is used. Returns ``(H,W)`` for 1 channel, ``(H,W,C)`` otherwise.
    """
    import numpy as np
    glue = _glue()
    rows, cols = int(mat.rows), int(mat.cols)
    ch = int(mat.channels())
    step = int(glue.mat_step(mat))
    addr = int(glue.mat_data_addr(mat))
    total = rows * step
    buf = (ctypes.c_uint8 * total).from_address(addr)
    view = np.frombuffer(buf, dtype=np.uint8).reshape(rows, step)
    view = view[:, : cols * ch]
    view = view.reshape(rows, cols, ch) if ch > 1 else view.reshape(rows, cols)
    if copy:
        return view.copy()
    cppyy_kit.keep_alive(buf, mat)
    return view


def to_gray(mat):
    """Return a single-channel 8-bit Mat: the input if already gray, else a
    ``cv::cvtColor`` conversion (BGR assumed for 3-channel, the ROS/OpenCV default).
    ORB needs a grayscale image."""
    cv = bringup_cv()
    if int(mat.channels()) == 1:
        return mat
    out = cv.Mat()
    cv.cvtColor(mat, out, cv.COLOR_BGR2GRAY)
    return out


class OrbDetector:
    """Thin wrapper over ``cv::ORB`` (CPU) or ``cv::cuda::ORB`` (GPU) with a single
    branch point in ``detect_and_compute``, so the CPU/GPU choice is one ``if``.

    Mirrors OpenCV: construct via :func:`create_orb`, then call
    ``detect_and_compute(mat)`` -> ``(keypoints, descriptors)`` where ``keypoints``
    is a ``std::vector<cv::KeyPoint>`` and ``descriptors`` an ``Nx32 CV_8U`` Mat.
    """

    def __init__(self, orb, use_cuda):
        self._orb = orb
        self._use_cuda = use_cuda
        self._cv = bringup_cv()

    @property
    def use_cuda(self):
        return self._use_cuda

    def detect_and_compute(self, mat, mask=None):
        cv = self._cv
        kps = cppyy.gbl.std.vector[cv.KeyPoint]()
        desc = cv.Mat()
        m = mask if mask is not None else cv.Mat()
        if self._use_cuda:
            # GPU branch: upload, detect, download. Exercised only when a CUDA
            # OpenCV build is present (cuda_available()); coded so it activates
            # with no other change to callers.
            g_img = cv.cuda.GpuMat()
            g_img.upload(mat)
            g_desc = cv.cuda.GpuMat()
            self._orb.detectAndCompute(g_img, cv.cuda.GpuMat(), kps, g_desc)
            g_desc.download(desc)
        else:
            self._orb.detectAndCompute(mat, m, kps, desc)
        return kps, desc


def create_orb(nfeatures=1000, use_cuda=None):
    """Create an :class:`OrbDetector`. ``use_cuda=None`` auto-selects the GPU path
    iff :func:`cuda_available` (here: always CPU). Pass ``use_cuda=False`` to force
    CPU. ``nfeatures`` is the max features per frame (``cv::ORB::create`` default is
    500; loop closure benefits from more)."""
    cv = bringup_cv()
    want_cuda = cuda_available() if use_cuda is None else use_cuda
    if want_cuda:
        if not cuda_available():
            raise RuntimeError("cv_kit: use_cuda=True but no CUDA OpenCV build present")
        orb = cv.cuda.ORB.create(nfeatures)
    else:
        with cppyy_kit.first_use("cv_kit.create_orb", "cv_kit.warmup()"):
            orb = cv.ORB.create(nfeatures)
    return OrbDetector(orb, want_cuda)


def descriptors_to_numpy(desc):
    """Copy an ``Nx32 CV_8U`` ORB descriptor Mat to an ``(N,32)`` uint8 NumPy
    array (one C++ memcpy). Empty Mat -> ``(0,32)``."""
    import numpy as np
    glue = _glue()
    n, cols = int(desc.rows), int(desc.cols)
    if n == 0:
        return np.empty((0, 32), dtype=np.uint8)
    out = np.empty((n, cols), dtype=np.uint8)
    glue.copy_u8_mat(desc, out.ctypes.data)
    return out


def keypoints_to_numpy(kps):
    """Copy keypoint pixel coordinates to an ``(N,2)`` float32 NumPy array
    (x, y) -- for e.g. a Rerun ``Points2D`` overlay."""
    import numpy as np
    glue = _glue()
    n = int(kps.size())
    if n == 0:
        return np.empty((0, 2), dtype=np.float32)
    out = np.empty((n, 2), dtype=np.float32)
    glue.keypoints_xy(kps, out.ctypes.data)
    return out


def warmup(nfeatures=1000):
    """Front-load cv_kit's one-time first-use JIT (the first ``cv::ORB::create`` +
    ``detectAndCompute`` call-wrapper codegen) on a throwaway synthetic frame, so
    the first live frame is steady-state. Call once during init."""
    import numpy as np
    bringup_cv()

    def _exercise():
        img = np.zeros((64, 64), dtype=np.uint8)
        img[16:48, 16:48] = 255
        det = create_orb(nfeatures)
        kps, desc = det.detect_and_compute(numpy_to_mat(img))
        descriptors_to_numpy(desc)
        keypoints_to_numpy(kps)

    cppyy_kit.warmup(_exercise)

"""
rclcppyy.tf -- drive the tf2 C++ transform stack from Python via cppyy.

tf2 is core ROS 2 (it ships in the default ``ros-base`` env, like rclcpp), so this
lives in rclcppyy-proper rather than in an opt-in ``kit``. The point is efficiency:
the stock ``tf2_ros`` **Python** ``TransformListener`` subscribes to ``/tf`` /
``/tf_static`` with **Python** callbacks, so every incoming ``TFMessage`` is
deserialized into Python objects and then fed **one TransformStamped at a time**
across the Python->C boundary into the ``tf2_py`` C-extension buffer -- all on a
Python thread holding the GIL (see ``tf2_ros/transform_listener.py`` ::``callback``
and ``tf2_ros/buffer.py`` ::``set_transform``). Under a busy tf tree that ingest cost
is entirely Python.

This module instead runs tf2's **C++** ``tf2_ros::TransformListener`` on its own
dedicated C++ thread against a ``tf2::BufferCore``: transforms are ingested wholly in
C++ (no per-message Python crossing, no GIL), and Python only reaches across when it
actually calls ``lookup_transform``. See docs/tf/REPORT.md for the mechanism and the
benchmark.

Usage::

    import rclcppyy
    from rclcppyy import tf

    rclcppyy.bringup_rclcpp()               # rclcpp up (tf bringup does this too)
    listener = tf.TransformListener()       # own node + own C++ spin thread
    ts = listener.lookup_transform("world", "sensor", timeout=1.0)
    print(ts.transform.translation.x, ts.transform.translation.y)

Design notes / friction hidden (mirror-don't-sugar otherwise):
    * The buffer is a plain ``tf2::BufferCore`` -- ``tf2_ros::Buffer``'s heavily
      overloaded ``lookupTransform``/``canTransform`` (rclcpp::Time + timeout forms)
      mis-resolve under cppyy and crash (a ``NodeT&&``-defaulted template ctor and an
      rclcpp::Clock dependency); BufferCore's single ``TimePoint`` overloads resolve
      cleanly. Timeouts are provided by a small C++ poll helper instead.
    * The ``Buffer`` and ``TransformListener`` are built in a ``cppdef`` factory, not
      constructed from Python: the listener's node-templated ctor + ``make_shared``
      don't resolve from Python (the recurring cppyy pattern -- build the object in
      C++).
    * ``lookup_transform`` returns the real ``geometry_msgs::msg::TransformStamped``
      (the same cppyy proxy the rest of rclcppyy uses); read ``.transform.translation``
      / ``.transform.rotation`` exactly as in C++.
"""
import os

import cppyy

# Import the functions directly: rclcppyy/__init__.py binds the *function*
# ``bringup_rclcpp`` into the package namespace, shadowing the submodule of the same
# name, so ``from rclcppyy import bringup_rclcpp`` would give the function, not the
# module.
from rclcppyy.bringup_rclcpp import bringup_rclcpp as _bringup_rclcpp
from rclcppyy.bringup_rclcpp import add_ros2_include_paths as _add_ros2_include_paths
from rclcppyy.kits import cppyy_kit

_TF_LIBS = (
    "libtf2.so",            # tf2::BufferCore -- the transform cache + math
    "libtf2_ros.so",        # tf2_ros::TransformListener -- the /tf, /tf_static listener
    "libtf2_msgs__rosidl_typesupport_cpp.so",
)

# BufferCore (the cache/math) + the C++ listener + the TFMessage type. We deliberately
# do NOT include tf2_ros/buffer.h: tf2_ros::Buffer's overload soup mis-resolves under
# cppyy (see module docstring); the listener only needs a tf2::BufferCore&.
_TF_HEADERS = (
    "tf2/buffer_core.hpp",
    "tf2_ros/transform_listener.h",
    "tf2_msgs/msg/tf_message.hpp",
)

# Factory + unambiguous accessors, compiled once. The listener's node-templated ctor
# and make_shared do not resolve from Python, and BufferCore's TimePoint overloads are
# the ones that resolve cleanly -- so construction and every lookup/canTransform go
# through these plain C++ free functions.
_CPP_GLUE = r"""
#include <thread>
#include <chrono>
#include <memory>
#include <string>
namespace rclcppyy_tf {

// Build the C++ listener attached to an existing rclcpp node (own dedicated spin
// thread when spin=true). Ingests /tf + /tf_static entirely in C++.
inline std::shared_ptr<tf2_ros::TransformListener>
make_listener(tf2::BufferCore& buf, rclcpp::Node::SharedPtr node, bool spin) {
  return std::make_shared<tf2_ros::TransformListener>(buf, node, spin);
}
// Build the listener with its own internally-created node (+ spin thread).
inline std::shared_ptr<tf2_ros::TransformListener>
make_listener_own(tf2::BufferCore& buf, bool spin) {
  return std::make_shared<tf2_ros::TransformListener>(buf, spin);
}

inline tf2::TimePoint timepoint_from_nanos(long long ns) {
  return tf2::TimePoint(std::chrono::nanoseconds(ns));
}

inline bool can(tf2::BufferCore& buf, const std::string& target,
                const std::string& source, const tf2::TimePoint& tp) {
  return buf.canTransform(target, source, tp);
}
inline geometry_msgs::msg::TransformStamped
lookup(tf2::BufferCore& buf, const std::string& target,
       const std::string& source, const tf2::TimePoint& tp) {
  return buf.lookupTransform(target, source, tp);
}
// Poll canTransform until it succeeds or the timeout elapses. The listener's
// dedicated C++ thread keeps ingesting while we wait (this is what
// setUsingDedicatedThread enables). steady_clock deadline; no rclcpp::Clock needed.
inline bool can_wait(tf2::BufferCore& buf, const std::string& target,
                     const std::string& source, const tf2::TimePoint& tp,
                     double timeout_sec) {
  auto deadline = std::chrono::steady_clock::now()
      + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout_sec));
  while (std::chrono::steady_clock::now() < deadline) {
    if (buf.canTransform(target, source, tp)) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return buf.canTransform(target, source, tp);
}
}  // namespace rclcppyy_tf
"""

_DONE = False
_TF2 = None
_GLUE = None


class TransformException(Exception):
    """A tf2 lookup/can-transform failure, with the C++ ``what()`` cleaned up."""


def bringup_tf():
    """Bring up the tf2 C++ stack under cppyy and return ``(tf2_namespace, glue)``.

    Ensures rclcpp is up (``bringup_rclcpp``), adds the ROS + tf2 include paths,
    JIT-includes ``tf2/buffer_core.hpp`` + the C++ transform listener + the TFMessage
    header, loads ``libtf2``/``libtf2_ros`` so symbols resolve, and compiles the
    construction/accessor glue. Idempotent.
    """
    global _DONE, _TF2, _GLUE
    if _DONE:
        return _TF2, _GLUE
    _bringup_rclcpp()
    _add_ros2_include_paths()
    conda = os.environ["CONDA_PREFIX"]
    cppyy.add_include_path(os.path.join(conda, "include"))
    for header in _TF_HEADERS:
        cppyy.include(header)
    cppyy_kit.load_libraries(_TF_LIBS, [os.path.join(conda, "lib")])
    cppyy.cppdef(_CPP_GLUE)
    _TF2 = cppyy.gbl.tf2
    _GLUE = cppyy.gbl.rclcppyy_tf
    _DONE = True
    return _TF2, _GLUE


def time_from_sec(seconds):
    """A ``tf2::TimePoint`` from seconds-since-epoch (``0.0`` -> latest available)."""
    tf2, _ = bringup_tf()
    return tf2.timeFromSec(float(seconds))


def duration_from_sec(seconds):
    """A ``tf2::Duration`` (``std::chrono::nanoseconds``) from seconds."""
    tf2, _ = bringup_tf()
    return tf2.durationFromSec(float(seconds))


def _to_timepoint(time):
    """Coerce a Python-side time to a ``tf2::TimePoint``.

    ``None`` -> ``TimePointZero`` (latest available); an ``int``/``float`` ->
    seconds-since-epoch; anything exposing ``.nanoseconds`` (rclpy/rclcpp ``Time``,
    callable or attribute) -> that instant; an existing ``tf2::TimePoint`` passes
    through.
    """
    tf2, glue = bringup_tf()
    if time is None:
        return tf2.TimePointZero
    if isinstance(time, (int, float)):
        return glue.timepoint_from_nanos(int(time * 1e9))
    nanos = getattr(time, "nanoseconds", None)
    if nanos is not None:
        ns = nanos() if callable(nanos) else nanos
        return glue.timepoint_from_nanos(int(ns))
    # Assume it is already a tf2::TimePoint proxy.
    return time


def _cpp_node(node):
    """Return the underlying ``rclcpp::Node::SharedPtr`` for ``node``.

    Accepts a raw cppyy ``rclcpp.Node`` (shared_ptr) or an rclcppyy node exposing
    ``_rclcpp_node``. ``None`` means "let the listener create its own node".
    """
    if node is None:
        return None
    inner = getattr(node, "_rclcpp_node", None)
    return inner if inner is not None else node


class TransformListener:
    """A tf2 transform listener whose ingest runs entirely in C++.

    Constructs a ``tf2::BufferCore`` and a ``tf2_ros::TransformListener`` that
    subscribes to ``/tf`` and ``/tf_static`` and, with ``spin_thread=True`` (the
    default), spins them on its **own dedicated C++ thread** -- so incoming transforms
    are decoded and inserted wholly in C++, with no per-message Python callback and no
    GIL contention. Python only crosses the boundary when you call
    ``lookup_transform`` / ``can_transform``.

    :param node: attach to this node (a cppyy ``rclcpp.Node`` or an rclcppyy node); if
        ``None``, the listener creates its own node internally.
    :param spin_thread: give the listener its own C++ executor thread (recommended --
        required for the ``timeout=`` wait in lookups/can_transform to make progress).
    :param cache_time_sec: BufferCore history length; ``None`` uses tf2's default (10s).
    """

    def __init__(self, node=None, *, spin_thread=True, cache_time_sec=None):
        tf2, glue = bringup_tf()
        self._tf2 = tf2
        self._glue = glue
        if cache_time_sec is None:
            self.buffer = tf2.BufferCore()
        else:
            self.buffer = tf2.BufferCore(duration_from_sec(cache_time_sec))
        cpp_node = _cpp_node(node)
        if cpp_node is None:
            self._listener = glue.make_listener_own(self.buffer, bool(spin_thread))
        else:
            self._listener = glue.make_listener(self.buffer, cpp_node, bool(spin_thread))
        self._closed = False
        # Release the listener (its dtor cancels the executor + joins the C++ thread)
        # before rclcpp shuts down. Registered LIFO-before shutdown_rclcpp (which
        # bringup registered first), so it runs first -- the correct order.
        cppyy_kit.register_teardown(self.close)

    def can_transform(self, target_frame, source_frame, time=None, timeout=0.0):
        """True if the transform ``target_frame <- source_frame`` is available at
        ``time`` (``None``/``0.0`` = latest). With ``timeout`` > 0, wait up to that
        many seconds for it to become available (the C++ listener keeps ingesting
        meanwhile)."""
        tp = _to_timepoint(time)
        if timeout and timeout > 0.0:
            return bool(self._glue.can_wait(self.buffer, target_frame, source_frame,
                                            tp, float(timeout)))
        return bool(self._glue.can(self.buffer, target_frame, source_frame, tp))

    def lookup_transform(self, target_frame, source_frame, time=None, timeout=0.0):
        """Return the ``geometry_msgs::msg::TransformStamped`` for ``target_frame <-
        source_frame`` at ``time`` (``None`` = latest).

        With ``timeout`` > 0, block up to that many seconds for the transform to
        arrive first. Raises :class:`TransformException` (with the tf2 ``what()``
        message) if the transform is unavailable or the wait times out.
        """
        tp = _to_timepoint(time)
        if timeout and timeout > 0.0:
            if not self._glue.can_wait(self.buffer, target_frame, source_frame,
                                       tp, float(timeout)):
                raise TransformException(
                    "timed out after %.3fs waiting for transform %r <- %r"
                    % (timeout, target_frame, source_frame))
        try:
            return self._glue.lookup(self.buffer, target_frame, source_frame, tp)
        except Exception as exc:  # tf2::LookupException / ExtrapolationException / ...
            raise TransformException(cppyy_kit.pretty_cpp_error(exc)) from None

    def set_transform(self, transform_stamped, authority="default_authority",
                      is_static=False):
        """Insert a transform directly into the buffer (bypassing the network) --
        handy for seeding a known tree in tests/benchmarks. ``transform_stamped`` is a
        ``geometry_msgs::msg::TransformStamped`` (cppyy proxy)."""
        return self.buffer.setTransform(transform_stamped, authority, bool(is_static))

    def get_frame_names(self):
        """List of frames currently in the buffer (as ``str``)."""
        return [f.decode() if isinstance(f, bytes) else str(f)
                for f in self.buffer.getAllFrameNames()]

    def all_frames_as_string(self):
        """Human-readable dump of the cached frame tree."""
        return str(self.buffer.allFramesAsString())

    def all_frames_as_yaml(self):
        """YAML dump of the cached frame tree (for debugging tools)."""
        return str(self.buffer.allFramesAsYAML())

    def close(self):
        """Release the C++ listener (cancels its executor + joins its thread).
        Idempotent; also registered to run at interpreter teardown."""
        if self._closed:
            return
        self._closed = True
        self._listener = None
        import gc
        gc.collect()

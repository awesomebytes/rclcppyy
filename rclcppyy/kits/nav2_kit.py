"""
nav2_kit -- compose your own Nav stack from Nav2's algorithm cores, in Python, via
cppyy.

Nav2's Python story is client-side only: ``nav2_simple_commander`` sends goals to
the C++ lifecycle servers; every algorithm (planners, controllers, costmap layers)
is a C++ class behind pluginlib. This kit takes the other road -- it drives Nav2's
**algorithm cores directly** from Python, with **no lifecycle servers, no
pluginlib, no tf**: Python owns the loop, C++ owns the math. It mirrors the
libraries' own C++ API against the installed Nav2, JIT-including the headers; there
is no binding to generate.

Two Nav2 cores are cleanly separable and surfaced here (see docs/nav2_kit/REPORT.md
for the full probe matrix, including what is *not*):

  * ``nav2_costmap_2d::Costmap2D`` -- a PLAIN grid class (no node): construct it,
    set costs, read the underlying ``unsigned char`` charmap. The kit adds a
    single-``memcpy`` NumPy<->charmap bridge (the per-cell ``setCost`` loop is
    ~600-3600x slower at 512x512/1024x1024 -- the same bulk-data lesson as pcl_kit).
  * ``nav2_navfn_planner::NavFn`` -- the NavFn Dijkstra/A* planner *algorithm*, which
    operates on the costmap char array with no node at all. The kit wraps its real
    friction: ``calcNavFnAstar`` only builds the potential field, so a plan needs a
    following ``calcPath``; start/goal cross as ``int*``; and the path comes back as
    raw ``float*`` X/Y arrays + a length.

Minimal plan on a synthetic world::

    import numpy as np
    from rclcppyy.kits import nav2_kit
    costmap_ns, navfn_ns = nav2_kit.bringup_nav2()

    grid = np.zeros((200, 200), dtype=np.uint8)      # 0 = free
    grid[80:120, 100] = nav2_kit.LETHAL_OBSTACLE     # a wall
    costmap = nav2_kit.costmap_from_numpy(grid, resolution=0.05)
    path = nav2_kit.plan_navfn(costmap, start=(20, 100), goal=(180, 100))  # (N,2) cells

Coordinate convention (kept consistent end to end): a NumPy grid is ``(H, W)`` =
``(rows=y, cols=x)``, indexed ``grid[y, x]``; a costmap cell is ``(mx=x, my=y)``;
NavFn path coordinates are ``(x, y)`` in cells. This is the same row-major layout as
a ``nav_msgs/OccupancyGrid`` (``data[y*W + x]``), so a plan lines up with a published
grid with no flip.

Notes / limits (v0):
    * This is the *algorithm-core* road, not a Nav2 stack: no lifecycle nodes, no
      pluginlib, no tf, no dynamic obstacle/inflation layers, no recovery behaviors.
    * NavFn's ``setCostmap(..., isROS=True)`` rescales ROS cost values (0-254) into
      NavFn's internal band and adds an obstacle border, exactly as the Nav2 server
      does; costs are the ``nav2_costmap_2d`` values (``nav2_kit.LETHAL_OBSTACLE`` ...).
    * Smac (Hybrid-A*) is NOT surfaced: its ``a_star.hpp`` transitively needs OMPL
      headers (absent from the nav2 env) and its collision checker is lifecycle-
      coupled -- see the REPORT. RegulatedPurePursuit's controller plugin is likewise
      lifecycle-coupled; only its header-only regulation math is separable.
    * cppyy returns ``unsigned char`` as a 1-char Python ``str`` -- read a single cell
      with ``ord(costmap.getCost(mx, my))``; the bulk ``costmap_to_numpy`` path avoids
      this.
"""
import os

import cppyy

from rclcppyy.kits import cppyy_kit

# The two .so whose symbols the cores resolve against. NavFn's setCost/getCharMap
# are undefined in libnav2_navfn_planner.so (they live in the costmap core), so both
# must be loaded; cppyy finds a symbol's owning library by scanning its search path
# at call time (add_library_path alone does not resolve symbols -- see cppyy_kit).
_NAV2_LIBS = (
    "libnav2_costmap_2d_core.so",
    "libnav2_navfn_planner.so",
)

# Headers: the cost-value constants, the plain Costmap2D grid class, and the NavFn
# planner algorithm. costmap_2d.hpp transitively pulls geometry_msgs / nav_msgs
# message headers, so the ROS include paths must be on the path first (bringup does
# this via rclcppyy's add_ros2_include_paths).
_NAV2_HEADERS = (
    "nav2_costmap_2d/cost_values.hpp",
    "nav2_costmap_2d/costmap_2d.hpp",
    "nav2_navfn_planner/navfn.hpp",
)

# C++ glue compiled once at bringup. The bulk-data copies and the NavFn call
# sequence live here on the C++ side (a per-cell Python loop is ~600-3600x slower,
# and building buffers from Python risks a cppyy SIGSEGV -- see cppyy_kit / pcl_kit).
# Addresses cross as uintptr_t and are reinterpret_cast'd in C++.
_CPP_GLUE = r"""
#include <cstring>
#include <cstdint>
namespace rclcppyy_nav2kit {
using nav2_costmap_2d::Costmap2D;
using nav2_navfn_planner::NavFn;

// numpy(uint8, row-major) -> costmap charmap: one std::memcpy (charmap is a plain
// uint8 buffer of size_x*size_y, same row-major layout as the numpy grid).
inline void load_charmap(Costmap2D& cm, uintptr_t src, std::size_t n) {
  std::memcpy(cm.getCharMap(), reinterpret_cast<const void*>(src), n);
}
// costmap charmap -> numpy(uint8): one std::memcpy out.
inline void dump_charmap(const Costmap2D& cm, uintptr_t dst, std::size_t n) {
  std::memcpy(reinterpret_cast<void*>(dst), cm.getCharMap(), n);
}

// NavFn plan, start..goal order. calcNavFnAstar only propagates the potential
// field; the path must then be traced with calcPath (this is the real NavFn
// friction). setStart/setGoal take int[2]. cancel checker is a C++ no-op. Returns
// the path length in points (0 if no plan). Path is left in NavFn's getPathX/Y.
// NOTE: setNavArr/setCostmap are done by the caller *before* this (setNavArr resets
// the cost array, so it must precede setCostmap).
inline int navfn_plan(NavFn& nav, int sx, int sy, int gx, int gy, bool allow_unknown) {
  int start[2] = {sx, sy};
  int goal[2] = {gx, gy};
  nav.setStart(start);
  nav.setGoal(goal);
  if (!nav.calcNavFnAstar([](){return false;})) return 0;
  return nav.calcPath(nav.nx * nav.ny / 2);
}
// NavFn path (subpixel cell coords) -> caller numpy float32 (N,2), start..goal.
inline int copy_path(NavFn& nav, uintptr_t xy_dst) {
  int n = nav.getPathLen();
  float* px = nav.getPathX();
  float* py = nav.getPathY();
  float* d = reinterpret_cast<float*>(xy_dst);
  for (int i = 0; i < n; ++i) { d[2 * i] = px[i]; d[2 * i + 1] = py[i]; }
  return n;
}
}  // namespace rclcppyy_nav2kit
"""

# Cost-value constants (nav2_costmap_2d), exposed as plain ints. cppyy would present
# these unsigned-char constants as 1-char strings; a plain int is what a user wants.
LETHAL_OBSTACLE = 254
INSCRIBED_INFLATED_OBSTACLE = 253
MAX_NON_OBSTACLE = 252
NO_INFORMATION = 255
FREE_SPACE = 0

_COSTMAP_NS = None
_NAVFN_NS = None
_DONE = False


def _ensure(with_ros_paths=True):
    """Bring up the Nav2 cores (headers + .so set + NumPy/NavFn glue). Idempotent."""
    global _COSTMAP_NS, _NAVFN_NS, _DONE
    if _DONE:
        return
    conda = os.environ["CONDA_PREFIX"]
    # costmap_2d.hpp needs the ROS message headers; add_ros2_include_paths registers
    # every ament package's include dir (cheap -- it adds paths, it does not JIT
    # rclcpp). $CONDA_PREFIX/include covers the nav2_navfn_planner/navfn.hpp
    # cross-reference layout.
    if with_ros_paths:
        from rclcppyy.bringup_rclcpp import add_ros2_include_paths
        add_ros2_include_paths()
    cppyy.add_include_path(os.path.join(conda, "include"))
    for header in _NAV2_HEADERS:
        cppyy.include(header)
    cppyy_kit.load_libraries(_NAV2_LIBS, [os.path.join(conda, "lib")])
    cppyy.cppdef(_CPP_GLUE)
    _COSTMAP_NS = cppyy.gbl.nav2_costmap_2d
    _NAVFN_NS = cppyy.gbl.nav2_navfn_planner
    _DONE = True


def bringup_nav2():
    """
    Bring up Nav2's Costmap2D + NavFn cores under cppyy and return the
    ``(nav2_costmap_2d, nav2_navfn_planner)`` namespaces. Idempotent.

    Adds the ROS message + Nav2 include paths, JIT-includes the cost-value / costmap
    / navfn headers, loads ``libnav2_costmap_2d_core.so`` + ``libnav2_navfn_planner.so``
    so calls resolve without ``LD_LIBRARY_PATH``, and compiles the NumPy/NavFn C++
    glue. Use the libraries' own API on the returned namespaces directly
    (``nav2_costmap_2d.Costmap2D``, ``nav2_navfn_planner.NavFn``).
    """
    _ensure()
    return _COSTMAP_NS, _NAVFN_NS


def _glue():
    _ensure()
    return cppyy.gbl.rclcppyy_nav2kit


def costmap_from_numpy(grid, resolution=0.05, origin=(0.0, 0.0), default_value=0):
    """
    Build a ``nav2_costmap_2d::Costmap2D`` from an ``(H, W)`` ``uint8`` occupancy
    grid (rows = y, cols = x), loading it with a single ``std::memcpy``.

    ``grid`` values are ``nav2_costmap_2d`` costs (``nav2_kit.FREE_SPACE`` ...
    ``LETHAL_OBSTACLE`` / ``NO_INFORMATION``); any dtype is coerced to contiguous
    ``uint8``. ``resolution`` is meters/cell; ``origin`` is the ``(x, y)`` world
    position of the grid's lower-left corner. The copy is a single memcpy because the
    charmap is a plain ``size_x*size_y`` ``uint8`` buffer with the same row-major
    layout as the array -- a per-cell ``setCost`` loop is ~600-3600x slower (REPORT).
    Returns the costmap.
    """
    import numpy as np
    arr = np.ascontiguousarray(grid, dtype=np.uint8)
    if arr.ndim != 2:
        raise ValueError(f"expected an (H, W) grid, got shape {arr.shape}")
    h, w = arr.shape
    costmap_ns, _ = bringup_nav2()
    ox, oy = origin
    # First touch of the Costmap2D ctor + the glue call JIT-compiles cppyy wrappers;
    # the notice points at nav2_kit.warmup().
    with cppyy_kit.first_use("nav2_kit.costmap_from_numpy", "nav2_kit.warmup()"):
        costmap = costmap_ns.Costmap2D(w, h, float(resolution), float(ox), float(oy),
                                       int(default_value))
        _glue().load_charmap(costmap, arr.ctypes.data, arr.size)
    return costmap


def costmap_to_numpy(costmap):
    """
    Extract a ``Costmap2D`` charmap to an ``(H, W)`` ``uint8`` NumPy array (rows = y),
    with a single ``std::memcpy`` out. Returns a private copy (safe after the costmap
    is gone).
    """
    import numpy as np
    w = int(costmap.getSizeInCellsX())
    h = int(costmap.getSizeInCellsY())
    out = np.empty((h, w), dtype=np.uint8)
    _glue().dump_charmap(costmap, out.ctypes.data, out.size)
    return out


def plan_navfn(costmap, start, goal, allow_unknown=True):
    """
    Plan a path with NavFn over ``costmap`` from ``start`` to ``goal`` (both
    ``(mx, my)`` cell coordinates), returning the path as an ``(N, 2)`` ``float32``
    NumPy array of ``(x, y)`` cell coordinates (start..goal order), or ``None`` if no
    plan is found.

    Wraps NavFn's real friction: it builds a ``NavFn`` sized to the costmap, feeds it
    the costmap char array with ``setCostmap(..., isROS=True, allow_unknown)``, runs
    ``calcNavFnAstar`` to propagate the potential field, then ``calcPath`` to trace
    the path (``calcNavFnAstar`` alone does *not* populate a path), and copies the raw
    ``float*`` X/Y arrays out. Coordinates are subpixel cells (~1/2-cell spacing);
    convert to world with ``costmap.mapToWorld`` or ``origin + (c+0.5)*resolution``.
    """
    import numpy as np
    _, navfn_ns = bringup_nav2()
    w = int(costmap.getSizeInCellsX())
    h = int(costmap.getSizeInCellsY())
    nav = navfn_ns.NavFn(w, h)
    # setNavArr (re)sizes and resets the cost array, so it must run BEFORE
    # setCostmap fills it (calling it after would wipe the costmap).
    nav.setNavArr(w, h)
    nav.setCostmap(costmap.getCharMap(), True, bool(allow_unknown))
    sx, sy = int(start[0]), int(start[1])
    gx, gy = int(goal[0]), int(goal[1])
    with cppyy_kit.first_use("nav2_kit.plan_navfn", "nav2_kit.warmup()"):
        n = int(_glue().navfn_plan(nav, sx, sy, gx, gy, bool(allow_unknown)))
    if n <= 0:
        return None
    path = np.empty((n, 2), dtype=np.float32)
    _glue().copy_path(nav, path.ctypes.data)
    # Pin the NavFn on the returned array so it can't be collected mid-extraction
    # (defensive; copy_path already ran, but keeps the pattern explicit).
    cppyy_kit.keep_alive(path, nav)
    return path


def warmup():
    """Front-load nav2_kit's one-time first-use JIT during init.

    The first ``costmap_from_numpy`` and first ``plan_navfn`` JIT-compile cppyy call
    wrappers for the Costmap2D ctor, the memcpy glue, and the NavFn plan/extract glue
    (a freeze/PCH does not remove this). This runs one throwaway build+plan on a tiny
    grid so the wrappers are cached process-globally before your first real call.
    """
    import numpy as np

    def _exercise():
        grid = np.zeros((16, 16), dtype=np.uint8)
        cm = costmap_from_numpy(grid, resolution=0.1)
        costmap_to_numpy(cm)
        plan_navfn(cm, (2, 2), (13, 13))

    cppyy_kit.warmup(_exercise)

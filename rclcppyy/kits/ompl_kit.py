"""
ompl_kit -- drive the Open Motion Planning Library (OMPL) from Python via cppyy.

OMPL's official Python bindings are generated with Py++/pygccxml: a multi-hour,
~6 GB-RAM build that lags upstream releases (the project is now migrating to
nanobind). This kit skips all of that -- it is a thin cppyy glue layer that
**mirrors OMPL's own C++ API**: you construct ``ob.RealVectorStateSpace``,
``og.SimpleSetup``, ``og.RRTConnect``, call ``setStateValidityChecker`` /
``setStartAndGoalStates`` / ``solve`` -- the same names and shapes as the official
C++ tutorials -- directly on the returned ``ompl`` namespace, against the OMPL
that is already installed. Nothing is generated.

The headline is **cross-language inheritance**: because OMPL's
``StateValidityChecker::isValid`` and ``OptimizationObjective::stateCost`` are
*plain* C++ virtuals (unlike BT.CPP's ``final`` ones, which blocked this), a
Python class can derive the C++ base and override the virtual, and the C++ planner
calls straight into Python -- thousands (or millions) of times per solve. See
docs/ompl_kit/REPORT.md for the mechanics and the measured cost.

A 2D plan with a Python validity checker, mirroring OMPL's first tutorial::

    from rclcppyy.kits import ompl_kit
    ob, og = ompl_kit.bringup_ompl()

    space = ob.RealVectorStateSpace(2)                 # OMPL's own API, verbatim
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0.0); bounds.setHigh(1.0)
    space.setBounds(bounds)

    ss = og.SimpleSetup(ob.StateSpacePtr(space))       # wrap transfers ownership
    ss.setStateValidityChecker(ompl_kit.validity_checker(lambda s: s[0] > 0.1))
    ...
    ss.setPlanner(ob.PlannerPtr(og.RRTConnect(ss.getSpaceInformation())))
    if ss.solve(1.0):
        print(ompl_kit.path_to_list(ss.getSolutionPath(), dim=2))

Two ways to give OMPL a validity checker, both real and both shown in the demos:
    * ``ompl_kit.validity_checker(fn)`` -- a Python function wrapped as the
      ``std::function<bool(const State*)>`` overload (built on ``cppyy_kit.callback``,
      so the signature is fixed and the lifetime pinned for you). Lowest ceremony.
    * a Python class deriving ``ob.StateValidityChecker`` (cross-inheritance): more
      OMPL-idiomatic, lets the checker hold state, and is what you subclass for an
      ``OptimizationObjective``. ``super().__init__(si)`` is required.

Notes / limits (v0):
    * State access: cppyy auto-downcasts the ``const State*`` a planner hands your
      checker to its concrete type (RTTI), so ``state[0]`` / ``state[1]`` work
      directly for a ``RealVectorStateSpace``. When you need an *explicit* cast (a
      compound space's ``state->as<SE2StateSpace::StateType>()``), use
      ``ompl_kit.as_state`` -- Python can't spell ``.as`` (it is a keyword).
    * The Python validity/cost callback holds the GIL; a single ``solve()`` is
      single-threaded so there is no contention, but it is ~170x slower per call
      than a native C++ checker (see REPORT). Prototype in Python, then *lower* the
      hot checker to a JIT'd C++ ``StateValidityChecker`` for production.
    * OMPL's ``ompl::RNG`` (used by every sampling planner) lives in the util
      namespace and must be seeded *before* the first sample; ``ompl_kit.set_seed``
      wraps it.
"""
import glob
import os

import cppyy

from rclcppyy.kits import cppyy_kit

# Base headers (always). SpaceInformation.h transitively pulls State/StateSpace;
# the rest are the spaces/objectives/RNG a typical plan touches. RandomNumbers.h
# is what set_seed reaches (ompl::RNG lives in util, not base).
_BASE_HEADERS = (
    "ompl/base/SpaceInformation.h",
    "ompl/base/StateValidityChecker.h",
    "ompl/base/OptimizationObjective.h",
    "ompl/base/ScopedState.h",
    "ompl/base/spaces/RealVectorStateSpace.h",
    "ompl/base/spaces/SE2StateSpace.h",
    "ompl/base/spaces/SE3StateSpace.h",
    "ompl/util/RandomNumbers.h",
)

# Geometric headers (default). SimpleSetup + the two RRT planners the demos use;
# any other planner is one more cppyy.include away (mirror, don't pre-wrap).
_GEOMETRIC_HEADERS = (
    "ompl/geometric/SimpleSetup.h",
    "ompl/geometric/planners/rrt/RRTConnect.h",
    "ompl/geometric/planners/rrt/RRTstar.h",
)

# The std::function<bool(const State*)> signature OMPL's setStateValidityChecker
# (Fn overload) wants. It is a *pointer* form; cppyy_kit.callback's type-hint
# inference would produce a `State&` reference instead, so the kit pins this
# explicitly (see REPORT -- callback() dogfood note).
_VALIDITY_SIG = "bool(const ompl::base::State*)"

_OMPL = None
_BASE_DONE = False
_GEOMETRIC_DONE = False


def _ompl_include_dir():
    """Locate the versioned OMPL include dir (include/ompl-<major>.<minor>)."""
    conda = os.environ.get("CONDA_PREFIX", "")
    dirs = sorted(glob.glob(os.path.join(conda, "include", "ompl-*")))
    if not dirs:
        raise RuntimeError(
            "No OMPL include dir (ompl-*) found under $CONDA_PREFIX/include. "
            "Install the ompl environment first: pixi install -e ompl"
        )
    return dirs[-1]


def _ensure_base():
    """Bring up OMPL's base layer (headers + libompl.so). Idempotent."""
    global _OMPL, _BASE_DONE
    if _BASE_DONE:
        return
    conda = os.environ["CONDA_PREFIX"]
    cppyy.add_include_path(_ompl_include_dir())
    cppyy.add_include_path(os.path.join(conda, "include", "eigen3"))
    for header in _BASE_HEADERS:
        cppyy.include(header)
    # cppyy resolves a symbol's owning .so at call time; load libompl.so
    # explicitly rather than relying on LD_LIBRARY_PATH (see cppyy_kit).
    cppyy_kit.load_libraries(["libompl.so"], [os.path.join(conda, "lib")])
    _OMPL = cppyy.gbl.ompl
    _BASE_DONE = True


def _ensure_geometric():
    """Add the geometric planners (SimpleSetup + RRT*). Idempotent; separated so a
    base-only user can skip the ~0.1 s SimpleSetup/planner JIT."""
    global _GEOMETRIC_DONE
    if _GEOMETRIC_DONE:
        return
    for header in _GEOMETRIC_HEADERS:
        cppyy.include(header)
    _GEOMETRIC_DONE = True


def bringup_ompl(with_geometric=True):
    """
    Bring up OMPL under cppyy and return ``(ompl.base, ompl.geometric)``. Idempotent.

    Discovers the OMPL install (``include/ompl-*`` + eigen3), JIT-includes the base
    headers (state spaces, validity checker, optimization objective) and -- with
    ``with_geometric=True`` (default) -- the geometric planners (SimpleSetup,
    RRTConnect, RRTstar), then loads libompl.so so calls resolve without
    LD_LIBRARY_PATH.

    Returns the ``ompl::base`` and ``ompl::geometric`` namespaces (the conventional
    ``ob`` / ``og`` aliases); use OMPL's own API on them directly. The full
    ``ompl`` namespace is also available as ``ompl_kit.ompl()``. When
    ``with_geometric=False`` the second element is ``None``.
    """
    _ensure_base()
    if with_geometric:
        _ensure_geometric()
    return _OMPL.base, (_OMPL.geometric if with_geometric else None)


def ompl():
    """The full ``cppyy.gbl.ompl`` namespace (base + geometric + util + ...)."""
    _ensure_base()
    return _OMPL


def validity_checker(fn, owner=None):
    """
    Wrap a Python ``fn(state) -> bool`` as OMPL's
    ``std::function<bool(const State*)>`` state-validity checker -- the low-ceremony
    alternative to subclassing ``ob.StateValidityChecker``.

    Pass the result straight to ``SpaceInformation.setStateValidityChecker`` /
    ``SimpleSetup.setStateValidityChecker``. The ``const State*`` cppyy hands ``fn``
    is auto-downcast (RTTI) to its concrete type, so for a ``RealVectorStateSpace``
    ``state[0]`` / ``state[1]`` work directly.

    Built on ``cppyy_kit.callback``: the signature is fixed to the pointer form OMPL
    wants and the wrapper + ``fn`` are pinned for you (the "callable was deleted"
    footgun is gone). ``owner`` pins them on that object's lifetime; without it they
    live for the process (see cppyy_kit.callback).
    """
    return cppyy_kit.callback(fn, signature=_VALIDITY_SIG, owner=owner)


def as_state(state, state_type):
    """
    OMPL's ``state->as<StateType>()`` downcast, spelled past Python's ``as``
    keyword (``state.as[...]`` is a SyntaxError).

    Usually unnecessary -- cppyy auto-downcasts the ``const State*`` a planner
    passes a validity/cost callback via RTTI, so ``state[i]`` already works for a
    ``RealVectorStateSpace``. Reach for this for an *explicit* cast, e.g. a compound
    space::

        se2 = ompl_kit.as_state(state, ob.SE2StateSpace.StateType)
        x, y, yaw = se2.getX(), se2.getY(), se2.getYaw()
    """
    return getattr(state, "as")[state_type]()


def set_seed(seed):
    """
    Seed OMPL's global RNG (``ompl::RNG::setSeed``) for reproducible planning.

    ``ompl::RNG`` lives in the *util* namespace (not ``base``), and OMPL warns +
    ignores a seed set *after* the first random sample -- so call this before
    constructing/solving. Seeding in-process across multiple solves is unreliable
    for the same reason; run each seeded solve in a fresh process.
    """
    _ensure_base()
    _OMPL.RNG.setSeed(int(seed))


def path_to_list(path, dim):
    """
    Extract a ``PathGeometric`` solution into a list of ``dim``-tuples of floats,
    one per waypoint.

    ``dim`` is the state-space dimension (the path does not carry it). Each state is
    auto-downcast, so ``state[j]`` reads coordinate ``j`` -- this just does the
    ``getStateCount``/``getState`` loop for you. For a compound space, read the
    waypoints yourself via ``as_state``.
    """
    out = []
    for i in range(path.getStateCount()):
        state = path.getState(i)
        out.append(tuple(float(state[j]) for j in range(dim)))
    return out

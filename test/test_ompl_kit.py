#!/usr/bin/env python3
"""Tests for rclcppyy.kits.ompl_kit (OMPL via cppyy).

OMPL is an optional dependency (the pixi `ompl` env), absent from the default env.
The whole module therefore auto-skips when the OMPL headers are not installed, so
the default `pixi run test` is unaffected. Run the real thing with
`pixi run -e ompl test-ompl`.

All tests share one process; bringup_ompl() is idempotent and each test builds its
own SimpleSetup, so they stay independent. Pure OMPL only (no rclcpp), so the
process exits cleanly. Planning is randomized -- tests assert feasibility and the
obstacle constraint, not exact paths (OMPL's global RNG can't be re-seeded mid
process).
"""
import glob
import os

import pytest

_HAVE_OMPL = bool(glob.glob(os.path.join(os.environ.get("CONDA_PREFIX", ""),
                                         "include", "ompl-*")))

pytestmark = pytest.mark.skipif(not _HAVE_OMPL,
                                reason="OMPL not installed (use the ompl env)")

if _HAVE_OMPL:
    from rclcppyy.kits import ompl_kit, cppyy_kit

OBSTACLE = (0.5, 0.5, 0.25)


def _clear(state):
    """True where `state` is outside the circular obstacle."""
    cx, cy, r = OBSTACLE
    return (state[0] - cx) ** 2 + (state[1] - cy) ** 2 > r ** 2


@pytest.fixture(scope="module")
def ompl():
    return ompl_kit.bringup_ompl()


def _setup(ob, og, checker_setter):
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0.0)
    bounds.setHigh(1.0)
    space.setBounds(bounds)
    ss = og.SimpleSetup(ob.StateSpacePtr(space))
    checker_setter(ss)
    start = ob.ScopedState[ob.RealVectorStateSpace](ss.getStateSpace())
    start[0], start[1] = 0.1, 0.1
    goal = ob.ScopedState[ob.RealVectorStateSpace](ss.getStateSpace())
    goal[0], goal[1] = 0.9, 0.9
    ss.setStartAndGoalStates(start, goal)
    ss.setPlanner(ob.PlannerPtr(og.RRTConnect(ss.getSpaceInformation())))
    return ss


def test_bringup_idempotent_and_namespaces(ompl):
    ob, og = ompl
    assert ob is ompl_kit.bringup_ompl()[0]           # idempotent
    assert hasattr(ob, "RealVectorStateSpace")
    assert hasattr(og, "SimpleSetup")
    assert ompl_kit.bringup_ompl(with_geometric=False)[1] is None


def test_pure_python_plan_via_callback(ompl):
    """Probe 2: a trivial always-true validity fn solves end to end."""
    ob, og = ompl
    calls = [0]

    def always_valid(state):
        calls[0] += 1
        return True

    ss = _setup(ob, og, lambda ss: ss.setStateValidityChecker(
        ompl_kit.validity_checker(always_valid, owner=ss)))
    assert bool(ss.solve(2.0))
    assert calls[0] > 0                               # C++ planner called Python
    path = ss.getSolutionPath()
    assert path.getStateCount() >= 2


def test_callback_respects_obstacle(ompl):
    """The Python validity fn actually constrains the plan (routes around)."""
    ob, og = ompl
    ss = _setup(ob, og, lambda ss: ss.setStateValidityChecker(
        ompl_kit.validity_checker(_clear, owner=ss)))
    assert bool(ss.solve(2.0))
    ss.simplifySolution()
    for x, y in ompl_kit.path_to_list(ss.getSolutionPath(), dim=2):
        assert (x - 0.5) ** 2 + (y - 0.5) ** 2 >= (0.25 ** 2) - 1e-6


def test_cross_inheritance_state_validity_checker(ompl):
    """HEADLINE: a Python class deriving ob.StateValidityChecker, isValid
    overridden in Python, called by the C++ planner."""
    ob, og = ompl

    class PyChecker(ob.StateValidityChecker):
        def __init__(self, si):
            super().__init__(si)                      # constructor chaining required
            self.calls = 0

        def isValid(self, state):
            self.calls += 1
            return _clear(state)

    holder = {}

    def set_checker(ss):
        chk = PyChecker(ss.getSpaceInformation())
        cppyy_kit.keep_alive(ss, chk)
        ss.setStateValidityChecker(ob.StateValidityCheckerPtr(chk))
        holder["chk"] = chk

    ss = _setup(ob, og, set_checker)
    assert bool(ss.solve(2.0))
    assert holder["chk"].calls > 0                    # the C++ planner hit Python
    for x, y in ompl_kit.path_to_list(ss.getSolutionPath(), dim=2):
        assert (x - 0.5) ** 2 + (y - 0.5) ** 2 >= (0.25 ** 2) - 1e-6


def test_optimization_objective_subclass_with_rrtstar(ompl):
    """Probe 5: a Python OptimizationObjective subclass drives RRTstar."""
    ob, og = ompl

    class PathLength(ob.OptimizationObjective):
        def __init__(self, si):
            super().__init__(si)
            self.si = si
            self.motion_calls = 0

        def stateCost(self, s):
            return ob.Cost(1.0)

        def motionCost(self, s1, s2):
            self.motion_calls += 1
            return ob.Cost(self.si.distance(s1, s2))

    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0.0)
    bounds.setHigh(1.0)
    space.setBounds(bounds)
    ss = og.SimpleSetup(ob.StateSpacePtr(space))
    ss.setStateValidityChecker(ompl_kit.validity_checker(lambda s: True, owner=ss))
    si = ss.getSpaceInformation()
    obj = PathLength(si)
    cppyy_kit.keep_alive(ss, obj)
    ss.setOptimizationObjective(ob.OptimizationObjectivePtr(obj))
    start = ob.ScopedState[ob.RealVectorStateSpace](ss.getStateSpace())
    start[0], start[1] = 0.0, 0.0
    goal = ob.ScopedState[ob.RealVectorStateSpace](ss.getStateSpace())
    goal[0], goal[1] = 1.0, 1.0
    ss.setStartAndGoalStates(start, goal)
    ss.setPlanner(ob.PlannerPtr(og.RRTstar(si)))
    assert bool(ss.solve(1.0))
    assert obj.motion_calls > 0                       # Python cost in the hot loop
    # RRTstar minimises length; the optimum here is the diagonal sqrt(2).
    assert ss.getSolutionPath().length() < 1.6


def test_path_to_list_shape(ompl):
    ob, og = ompl
    ss = _setup(ob, og, lambda ss: ss.setStateValidityChecker(
        ompl_kit.validity_checker(lambda s: True, owner=ss)))
    assert bool(ss.solve(2.0))
    pts = ompl_kit.path_to_list(ss.getSolutionPath(), dim=2)
    assert len(pts) >= 2
    assert all(len(p) == 2 and all(isinstance(c, float) for c in p) for p in pts)


def test_as_state_explicit_downcast(ompl):
    """as_state spells state->as<StateType>() past Python's `as` keyword."""
    ob, og = ompl
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0.0)
    bounds.setHigh(1.0)
    space.setBounds(bounds)
    si = ob.SpaceInformation(ob.StateSpacePtr(space))
    si.setup()
    st = si.allocState()
    rv = ompl_kit.as_state(st, ob.RealVectorStateSpace.StateType)
    rv[0], rv[1] = 0.3, 0.7
    assert rv[0] == pytest.approx(0.3)
    assert rv[1] == pytest.approx(0.7)
    si.freeState(st)


def test_set_seed_callable(ompl):
    """set_seed reaches ompl::RNG (util namespace) without error."""
    ompl_kit.set_seed(123)      # may warn if RNG already used this process; must not raise

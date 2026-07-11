#!/usr/bin/env python
"""
OMPL's first geometric-planning tutorial, in Python via rclcppyy's ompl_kit. This
mirrors the official C++ tutorial line-for-line -- same StateSpace + bounds, same
SimpleSetup, same setStateValidityChecker / setStartAndGoalStates / solve /
getSolutionPath -- only the state-validity checker is a Python function.

The problem: plan a 2D path in the unit square from (0.1, 0.1) to (0.9, 0.9)
around a circular obstacle of radius 0.25 centred at (0.5, 0.5). The straight line
would cut through the circle, so a valid plan must route around it.

Reference: https://ompl.kavrakilab.org/geometricPlanningSE3.html (RigidBodyPlanning)
Run:       pixi run -e ompl demo-ompl-plan
"""
from rclcppyy.kits import ompl_kit

ob, og = ompl_kit.bringup_ompl()

OBSTACLE = (0.5, 0.5, 0.25)  # centre x, centre y, radius


def is_state_valid(state):
    # cppyy auto-downcasts the const State* to RealVectorStateSpace::StateType,
    # so state[0] / state[1] read the coordinates directly (OMPL's C++ tutorial:
    # const auto* s = state->as<ob::RealVectorStateSpace::StateType>();).
    cx, cy, r = OBSTACLE
    return (state[0] - cx) ** 2 + (state[1] - cy) ** 2 > r ** 2


def main():
    ompl_kit.set_seed(42)  # reproducible plan

    space = ob.RealVectorStateSpace(2)                 # OMPL's own API, verbatim
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0.0)
    bounds.setHigh(1.0)
    space.setBounds(bounds)

    ss = og.SimpleSetup(ob.StateSpacePtr(space))       # wrap transfers ownership
    ss.setStateValidityChecker(ompl_kit.validity_checker(is_state_valid, owner=ss))

    ScopedState = ob.ScopedState[ob.RealVectorStateSpace]
    start = ScopedState(ss.getStateSpace())
    start[0], start[1] = 0.1, 0.1
    goal = ScopedState(ss.getStateSpace())
    goal[0], goal[1] = 0.9, 0.9
    ss.setStartAndGoalStates(start, goal)

    ss.setPlanner(ob.PlannerPtr(og.RRTConnect(ss.getSpaceInformation())))

    if ss.solve(1.0):
        ss.simplifySolution()
        path = ss.getSolutionPath()
        waypoints = ompl_kit.path_to_list(path, dim=2)
        print(f"Solved. Path length {path.length():.4f} over {len(waypoints)} waypoints:")
        for x, y in waypoints:
            dist = ((x - OBSTACLE[0]) ** 2 + (y - OBSTACLE[1]) ** 2) ** 0.5
            print(f"  ({x:.3f}, {y:.3f})   dist-to-obstacle-centre {dist:.3f}")
    else:
        print("No solution found.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python
"""
nav2_kit benchmark -- the "C++ math, Python orchestration" performance story.

Plan the SAME problem on NxN grids two ways and print a timing table:
  * NavFn  -- Nav2's real C++ planner algorithm, driven from Python via nav2_kit
    (the costmap load is a single memcpy; the plan never leaves C++);
  * py-A*  -- a plain pure-Python A* (numpy grid + heapq), written below and labeled
    as such, standing in for "what you'd write if you re-implemented the planner in
    Python".

Same start/goal, same "two rooms + doorway" world, 8-connected, so both do real
work. The point is not that A* == NavFn (they are different algorithms) but the
order-of-magnitude cost of keeping the search loop in Python vs handing the grid to
a compiled core. The pure-Python side is wall-time-capped and reports DNF past it.

Shared machine during measurement -- treat as directional, not exact.

Run: pixi run -e nav2 bench-nav2-plan
"""
import argparse
import heapq
import math
import time

import numpy as np

from rclcppyy.kits import nav2_kit

SQRT2 = math.sqrt(2.0)


def make_world(n):
    """NxN serpentine maze: horizontal walls with alternating left/right gaps, so the
    only route snakes down the whole grid. A straight-line heuristic is badly misled,
    forcing A* to expand a large fraction of the free cells -- a real search workload
    (a simple wall+doorway lets the heuristic walk straight to the goal, which is not
    an interesting comparison)."""
    g = np.zeros((n, n), dtype=np.uint8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = nav2_kit.LETHAL_OBSTACLE
    spacing = max(6, n // 10)
    gap = max(3, n // 20)
    for i, r in enumerate(range(spacing, n - spacing, spacing)):
        g[r:r + 2, :] = nav2_kit.LETHAL_OBSTACLE            # a 2-cell-thick wall
        if i % 2 == 0:
            g[r:r + 2, n - 1 - gap:n - 1] = nav2_kit.FREE_SPACE   # gap on the right
        else:
            g[r:r + 2, 1:1 + gap] = nav2_kit.FREE_SPACE           # gap on the left
    return g


def py_astar(grid, start, goal, budget_s):
    """A plain pure-Python 8-connected A* (numpy grid + heapq). start/goal are
    (x, y) cells. Returns (path_len_cells, expansions) or (None, expansions) if no
    path / the wall-time budget is exceeded. Deliberately straightforward -- this is
    the 'Python in the search loop' baseline."""
    h, w = grid.shape
    lethal = nav2_kit.INSCRIBED_INFLATED_OBSTACLE  # 253: treat >=253 as blocked
    sx, sy = start
    gx, gy = goal
    open_heap = [(0.0, sx, sy)]
    g_cost = np.full((h, w), np.inf, dtype=np.float64)
    g_cost[sy, sx] = 0.0
    closed = np.zeros((h, w), dtype=bool)
    neigh = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2))
    expansions = 0
    t_end = time.perf_counter() + budget_s
    while open_heap:
        _, x, y = heapq.heappop(open_heap)
        if x == gx and y == gy:
            return round(float(g_cost[gy, gx]), 1), expansions   # path cost (cell-steps)
        if closed[y, x]:
            continue
        closed[y, x] = True
        expansions += 1
        if expansions % 4096 == 0 and time.perf_counter() > t_end:
            return None, expansions            # DNF: over the wall-time budget
        base = g_cost[y, x]
        for dx, dy, step in neigh:
            nx, ny = x + dx, y + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            if grid[ny, nx] >= lethal or closed[ny, nx]:
                continue
            ng = base + step
            if ng < g_cost[ny, nx]:
                g_cost[ny, nx] = ng
                f = ng + math.hypot(gx - nx, gy - ny)   # Euclidean heuristic
                heapq.heappush(open_heap, (f, nx, ny))
    return None, expansions


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sizes", type=int, nargs="+", default=[256, 512, 1024])
    ap.add_argument("--py-budget", type=float, default=15.0,
                    help="wall-time cap (s) for the pure-Python A* per size")
    args = ap.parse_args()

    nav2_kit.bringup_nav2()
    nav2_kit.warmup()                          # move NavFn/costmap first-use JIT off row 1

    print(f"{'N':>6} | {'cells':>9} | {'NavFn C++ ms':>13} | {'py-A* ms':>11} | "
          f"{'py expansions':>13} | {'NavFn speedup':>13}")
    print("-" * 82)
    for n in args.sizes:
        grid = make_world(n)
        start, goal = (2, 2), (n - 3, n - 3)   # opposite corners, through the maze

        costmap = nav2_kit.costmap_from_numpy(grid)
        t0 = time.perf_counter()
        path = nav2_kit.plan_navfn(costmap, start, goal)
        t_navfn = (time.perf_counter() - t0) * 1e3
        navfn_ok = path is not None

        t0 = time.perf_counter()
        _, expansions = py_astar(grid, start, goal, args.py_budget)
        t_py = (time.perf_counter() - t0) * 1e3
        dnf = t_py > args.py_budget * 1e3 * 0.99

        py_str = "DNF" if dnf else f"{t_py:11.1f}"
        speed = "n/a" if dnf else f"{t_py / t_navfn:12.0f}x"
        navfn_str = f"{t_navfn:13.2f}" if navfn_ok else "     no-plan "
        print(f"{n:>6} | {n * n:>9} | {navfn_str} | {py_str:>11} | "
              f"{expansions:>13} | {speed:>13}")

    print("\nNavFn is the same C++ algorithm Nav2 ships; py-A* is the Python baseline "
          "in this file.\nDNF = pure-Python A* exceeded the wall-time budget "
          f"({args.py_budget:.0f}s) -- the point of the story.")


if __name__ == "__main__":
    main()

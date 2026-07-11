#!/usr/bin/env python
"""
nav2_kit demo 1 -- plan on a synthetic occupancy grid, with Nav2's real cores.

Build a small "two rooms + doorway" world as a NumPy grid, turn it into a real
``nav2_costmap_2d::Costmap2D`` (single memcpy), plan across it with the real
``nav2_navfn_planner::NavFn`` A* algorithm -- no lifecycle server, no pluginlib, no
tf -- and print the path. This is the whole "compose your own nav stack from Nav2's
algorithm cores in Python" thesis in ~20 lines.

Run: pixi run -e nav2 demo-nav2-plan
"""
import numpy as np

from rclcppyy.kits import nav2_kit

W = H = 100
RES = 0.05  # meters/cell


def make_world():
    """A 100x100 world: outer walls + a vertical divider with a doorway."""
    g = np.zeros((H, W), dtype=np.uint8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = nav2_kit.LETHAL_OBSTACLE  # border
    g[:, W // 2] = nav2_kit.LETHAL_OBSTACLE                             # divider
    g[H // 2 - 6:H // 2 + 6, W // 2] = nav2_kit.FREE_SPACE             # doorway
    return g


def main():
    nav2_kit.bringup_nav2()          # includes headers, loads the .so, compiles glue
    grid = make_world()
    costmap = nav2_kit.costmap_from_numpy(grid, resolution=RES)
    start, goal = (20, H // 2), (80, H // 2)   # left room -> right room
    path = nav2_kit.plan_navfn(costmap, start, goal)

    if path is None:
        print("No plan found.")
        return
    print(f"World {W}x{H} @ {RES} m/cell; divider at x={W // 2} with a doorway.")
    print(f"Planned {len(path)} waypoints from {start} to {goal} (cell coords):")
    # print a handful, evenly sampled, so the route around the doorway is visible
    step = max(1, len(path) // 8)
    for i in range(0, len(path), step):
        x, y = path[i]
        print(f"    ({x:6.1f}, {y:6.1f})")
    x, y = path[-1]
    print(f"    ({x:6.1f}, {y:6.1f})   <- goal")
    # the doorway is the only gap in the x=W/2 divider, so a valid path must pass it
    crossed = [p for p in path if abs(p[0] - W // 2) < 1.0]
    if crossed:
        ys = [p[1] for p in crossed]
        print(f"Path crosses the divider (x={W // 2}) at y in "
              f"[{min(ys):.1f}, {max(ys):.1f}] -- through the doorway.")


if __name__ == "__main__":
    main()
